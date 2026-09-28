"""Coupled research model. All material cable cells retain mass during payout.

Stored cells are held by compliant spool supports. Cells activate at the fairlead,
not by uniformly stretching the entire cable. This deliberately avoids remapping
mass/velocity at topology changes. The discretization is a bead-chain approximation
and its activation transients require mesh convergence checks.
"""
import numpy as np
from .math3d import rotation, qdot, quaternion, point_state, spring_force, contact_force, attitude_error, sphere_box_contact, cross


def schedule(rows, t):
    a = np.asarray(rows, float)
    value = np.interp(t, a[:, 0], a[:, 1])
    i = np.searchsorted(a[:, 0], t, side="right")-1
    rate = (a[i+1, 1]-a[i, 1])/(a[i+1, 0]-a[i, 0]) if 0 <= i < len(a)-1 else 0.
    return float(value), float(rate)


class CoupledModel:
    def __init__(self, cfg, aero, trim=None, phase="mission", fixed_length=None):
        from .config import validate
        validate(cfg)
        if phase not in ('mission','stowed','deployed','aircraft_only','recovery'):
            raise ValueError('Unknown simulation phase: '+str(phase))
        if fixed_length is not None and (not np.isfinite(fixed_length) or not cfg['winch']['stowed_length_m']<=fixed_length<=cfg['cable']['length_m']):
            raise ValueError('Fixed cable length outside physical bounds')
        aero.assert_compatible(cfg)
        self.c, self.aero = cfg, aero
        self.phase, self.fixed_length = phase, fixed_length
        self.n = int(cfg["cable"]["segments"])
        self.h = cfg["cable"]["length_m"]/self.n
        self.mn = cfg["cable"]["density_kg_m"]*self.h
        self.size = 26+6*self.n
        self.ma, self.ms = cfg["aircraft"]["mass_kg"], cfg["sensor"]["mass_kg"]
        self.Ia = np.asarray(cfg["aircraft"]["inertia_kgm2"], float)
        self.point_mass = cfg['sensor'].get('model') == 'point_mass'
        self.Is = None if self.point_mass else np.asarray(cfg["sensor"]["inertia_kgm2"], float)
        self.Iai = np.linalg.inv(self.Ia)
        self.Isi = None if self.point_mass else np.linalg.inv(self.Is)
        self.attach = np.asarray(cfg["aircraft"]["tow_point_m"], float)
        self.sensor_attach = np.asarray(cfg["sensor"]["tow_point_m"], float)
        self.trim = trim or {"elevator_deg": 0., "thrust_N": 0., "alpha_rad": 0.}
        self.controller = cfg['flight'].get('controller')
        if self.controller and self.controller.get('enabled'):
            from .control import validate_controller, resolve_controller
            validate_controller(cfg['flight'], cfg['aircraft'], cfg['aero'])
            self.controller = resolve_controller(cfg, aero, self.trim)
        self.captured = False
        self.capture_time = None
        self.active_override = None
        self.sensor_enabled = phase != "aircraft_only"
        self.last = {}
        self.mesh_contact_enabled = bool(cfg.get('collision', {}).get('enabled', False)) and self.sensor_enabled and not self.point_mass
        self._mesh_contacts = None
        self._parallel_rhs = None

    def close(self):
        if self._parallel_rhs is not None:
            self._parallel_rhs.close()
            self._parallel_rhs = None
        if self._mesh_contacts is not None:
            self._mesh_contacts.close()
            self._mesh_contacts = None

    def rhs_batch(self, t, states):
        """Columns are independent, with identical physical and event state."""
        workers = self.c.get('simulation', {}).get('jacobian_workers', 1)
        if workers == 1 or states.shape[1] == 1:
            return np.column_stack([self.rhs(t, state) for state in states.T])
        if self._parallel_rhs is None:
            from .parallel_rhs import ParallelRHS
            self._parallel_rhs = ParallelRHS(self, workers)
        return self._parallel_rhs(t, states)

    def contact_geometry(self, t, y, distance=None):
        if not self.mesh_contact_enabled:
            return []
        if self._mesh_contacts is None:
            from .collision import MeshContacts
            self._mesh_contacts = MeshContacts(self.c)
        a, s = y[:13], y[13:26]
        ra, rs = rotation(a[6:10]), rotation(s[6:10])
        center = ra.T @ (s[:3]-a[:3])
        k = self.active_count(self.length(t)[0]) if self.active_override is None else self.active_override
        nose = center + ra.T @ rs @ self.sensor_attach
        nodes = (y[26:].reshape(self.n,6)[:k,:3]-a[:3]) @ ra
        chain = np.vstack([nose,nodes,self.attach])
        return self._mesh_contacts.contacts(center, ra.T@rs, chain,
                          self.door_state(t)[0], distance)

    def clearance_metric(self, t, y):
        contact=self.c['collision']
        radius=max(contact['skin_m'],contact.get('clearance_query_m',contact['skin_m']))
        hits = self.contact_geometry(t,y,distance=radius)
        return min((h.get('geometry_gap',h['gap']) for h in hits), default=radius) - contact['minimum_gap_m']

    def length(self, t):
        if self.phase in ('stowed','aircraft_only'):
            return self.c['winch']['stowed_length_m'],0.
        return (self.fixed_length, 0.) if self.fixed_length is not None else schedule(self.c["winch"]["length_schedule"], t)

    def door_state(self, t):
        if self.phase=='deployed':
            return max(v for _,v in self.c['winch']['door_schedule']),0.
        if self.phase=='stowed':
            return 0.,0.
        rows = self.c['winch']['door_schedule']
        closing = self.door_closing_start()
        if closing is None or t < closing:
            return schedule(rows, t)
        if not self.captured:
            return schedule(rows, closing)[0], 0.
        release = max(closing, self.capture_time + self.c['winch'].get('door_capture_hold_s', 0.))
        if t < release:
            return schedule(rows, closing)[0], 0.
        return schedule(rows, t-(release-closing))

    def door_closing_start(self):
        """The closing command is delayed until the compliant latch engages."""
        if self.phase != 'mission' or not self.c['winch'].get('door_capture_interlock', False):
            return None
        rows = self.c['winch']['door_schedule']
        return next((a[0] for a,b in zip(rows[:-1], rows[1:]) if b[1] < a[1]), None)

    def door_transition_times(self):
        closing = self.door_closing_start()
        if closing is None or not self.captured:
            return []
        release = max(closing, self.capture_time + self.c['winch'].get('door_capture_hold_s', 0.))
        return [t+release-closing for t,_ in self.c['winch']['door_schedule'] if t >= closing]

    def active_count(self, length):
        return min(self.n, max(0, int(np.floor(length/self.h+.5))))

    def initial(self):
        c = self.c
        y = np.zeros(self.size)
        y[2] = -c["flight"]["altitude_m"]
        y[3] = c["flight"]["speed_m_s"]
        y[6:10] = quaternion(pitch=self.trim["alpha_rad"])
        a, s = y[:13], y[13:26]
        r = rotation(a[6:10])
        s[:3] = a[:3]+r@np.array(c["bay"]["stowed_center_m"])
        s[3:6] = a[3:6]
        s[6:10] = self.latch_orientation(a)
        nodes = y[26:].reshape(self.n, 6)
        p, v = point_state(a, self.attach)
        nodes[:, :3], nodes[:, 3:] = p, v
        # A separated fairlead can leave material cells outside the reel even
        # while the sensor is stowed. Place those cells along the real initial
        # span; stacking them at the reel would create artificial initial strain.
        length=self.length(0.)[0]
        active=self.active_count(length)
        if active:
            nose,nose_v=point_state(s,self.sensor_attach)
            from .routing import cable_initial_points
            nodes[:active,:3]=cable_initial_points(c,a,s,active,length)
            nodes[:active,3:]=a[3:6]+np.cross(r@a[10:13],nodes[:active,:3]-a[:3])
        return y

    def latch_orientation(self,a):
        from scipy.spatial.transform import Rotation
        matrix=rotation(a[6:10])@rotation(self.c['bay'].get('stowed_quaternion_wxyz',[1.,0.,0.,0.]))
        q=Rotation.from_matrix(matrix).as_quat()
        return np.r_[q[3],q[:3]]

    def wind(self, t):
        f = self.c["flight"]
        wind = np.array(f["wind_ned_m_s"], float)
        gust = f.get("gust")
        if gust and gust["start_s"] < t < gust["start_s"]+gust["duration_s"]:
            phase = (t-gust["start_s"])/gust["duration_s"]
            wind += np.asarray(gust["velocity_ned_m_s"])*np.sin(np.pi*phase)**2
        return wind

    def aircraft_loads(self, t, a):
        c = self.c
        r = rotation(a[6:10])
        vr = r.T@(a[3:6]-self.wind(t))
        speed = np.linalg.norm(vr)
        if speed < 1:
            raise ValueError("AERO_DOMAIN: airspeed below 1 m/s")
        alpha, beta = np.arctan2(vr[2], vr[0]), np.arcsin(np.clip(vr[1]/speed, -1, 1))
        el, thrust = self.trim["elevator_deg"], self.trim["thrust_N"]
        controls = c["flight"].get("controls", {})
        el += schedule(controls.get("elevator_delta_deg", [[0, 0], [1, 0]]), t)[0]
        thrust += schedule(controls.get("thrust_delta_N", [[0, 0], [1, 0]]), t)[0]
        control_diagnostics = {}
        if self.controller and self.controller.get('enabled'):
            from .control import longitudinal_commands
            pitch = np.arcsin(np.clip(-r[2, 0], -1., 1.))
            el, thrust, control_diagnostics = longitudinal_commands(self.controller, t, a, self.trim, pitch, speed, el, thrust)
        if not 0 <= thrust <= c['aircraft']['max_thrust_N']:
            raise ValueError('CONTROL_DOMAIN: thrust command outside limits')
        lateral = [schedule(controls.get(k, [[0, 0], [1, 0]]), t)[0] for k in ("aileron_deg", "rudder_deg")]
        coeff = self.aero.evaluate(alpha, beta, el, a[10:13], speed, lateral)
        area, chord, span = self.aero.refs
        dynamic = .5*c["flight"]["rho_kg_m3"]*speed**2
        force = dynamic*area*coeff[:3]
        force -= dynamic*area*c["aircraft"]["profile_cd"]*vr/speed
        moment = dynamic*area*coeff[3:]*np.array([span, chord, span])
        ref = np.array(self.aero.metadata["moment_reference_frd_m"])
        cg = np.array(c["aircraft"]["cg_m"])
        moment += cross(ref-cg, force)
        force += [thrust, 0, 0]
        moment += cross(np.asarray(c["aircraft"]["thrust_point_m"]), [thrust, 0, 0])
        return r@force, moment, {"alpha_deg": np.rad2deg(alpha), "beta_deg": np.rad2deg(beta), "airspeed_m_s": speed,
                                 "elevator_deg": el, "thrust_N": thrust, **control_diagnostics}

    def sensor_loads(self, t, a, s):
        if self.point_mass:
            return np.zeros(3), np.zeros(3), 1.
        c, sensor = self.c, self.c["sensor"]
        ra, rs = rotation(a[6:10]), rotation(s[6:10])
        local = ra.T@(s[:3]-a[:3])
        # Explicit low-order exposure model, not a resolved rear-door wake model.
        exposure = np.clip((c["bay"]["exit_x_m"]-local[0]+sensor["length_m"]/2)/sensor["length_m"], 0, 1)
        if self.phase == "deployed":
            exposure = 1.
        wake = c["flight"]["local_flow_factor"]
        vr = rs.T@(s[3:6]-self.wind(t))*wake
        speed = np.linalg.norm(vr)
        if speed < 1e-9 or exposure == 0:
            return np.zeros(3), np.zeros(3), float(exposure)
        alpha = np.arctan2(vr[2], vr[0])
        beta = np.arctan2(vr[1], np.hypot(vr[0], vr[2]))
        q = .5*c["flight"]["rho_kg_m3"]*speed**2*sensor["area_m2"]*exposure
        coeff = -sensor["cd"]*vr/speed
        coeff += np.array([0., -sensor["cy_beta"]*beta, -sensor["cz_alpha"]*alpha])
        force = q*coeff
        scales = np.array([sensor["span_m"], sensor["length_m"], sensor["span_m"]])
        moment = q*scales*np.array([0., sensor["cm_alpha"]*alpha, sensor["cn_beta"]*beta])
        moment -= q*scales*np.asarray(sensor["rate_damping"])*s[10:13]*scales/(2*speed)
        return rs@force, moment, float(exposure)

    def rhs(self, t, y, diagnostics=False):
        c = self.c
        a, s = y[:13], y[13:26]
        ra, rs = rotation(a[6:10]), rotation(s[6:10])
        nodes = y[26:].reshape(self.n, 6)
        g = np.array([0., 0., c["flight"]["g_m_s2"]])
        fa, ta, d = self.aircraft_loads(t, a)
        fa += self.ma*g
        fs, ts, exposure = self.sensor_loads(t, a, s) if self.sensor_enabled else (np.zeros(3), np.zeros(3), 0.)
        fs += self.ms*g
        fn = np.tile(self.mn*g, (self.n, 1))
        length, payout = self.length(t)
        k = self.active_count(length) if self.active_override is None else self.active_override
        anchor, anchor_v = point_state(a, self.attach)
        nose, nose_v = point_state(s, self.sensor_attach)
        cable = c["cable"]
        tensions = []
        support_power = 0.
        cable_exposure_mean=0.;internal_cable_drag=0.

        def apply_aircraft(f, p):
            nonlocal fa, ta
            fa += f
            ta += ra.T@cross(p-a[:3], f)

        def apply_sensor(f, p):
            nonlocal fs, ts
            fs += f
            ts += rs.T@cross(p-s[:3], f)

        if self.sensor_enabled:
            # Material nodes 0..k-1 are outside spool, counted from sensor toward spool.
            if k < self.n:
                delta = nodes[k:, :3]-anchor
                relv = nodes[k:, 3:]-anchor_v
                f = -cable["spool_k_N_m"]/self.n*delta-cable["spool_c_Ns_m"]/self.n*relv
                fn[k:] += f
                fa -= f.sum(axis=0)
                ta -= ra.T@cross(nodes[k:, :3]-a[:3], f).sum(axis=0)
            if k == 0:
                segment_damping=cable['damping_Ns_m']*cable['length_m']/max(length,self.h*.1)
                f, tension = spring_force(nose-anchor, nose_v-anchor_v, length, cable["EA_N"], segment_damping, regularization=self.h*.1, damping_strain=cable['damping_engagement_strain'])
                apply_aircraft(f, anchor)
                apply_sensor(-f, nose)
                tensions.append(tension)
            else:
                # Fixed material cells, last partial span changes ONLY at feed point.
                chain = np.vstack([nose,nodes[:k,:3],anchor])
                velocities = np.vstack([nose_v,nodes[:k,3:],anchor_v])
                rests = np.r_[self.h/2,np.full(k-1,self.h),max(0.,length-(k-.5)*self.h)]
                delta=np.diff(chain,axis=0);dist=np.linalg.norm(delta,axis=1)
                directions=delta/np.maximum(dist[:,None],1e-12)
                rate=(np.diff(velocities,axis=0)*directions).sum(axis=1)
                denominators=np.maximum(rests,self.h*.1)
                engagement=np.clip((dist-rests)/(denominators*cable['damping_engagement_strain']),0,1)
                tensions=np.maximum(0.,cable['EA_N']/denominators*(dist-rests)+engagement*cable['damping_Ns_m']*cable['length_m']/denominators*rate)
                tensions[dist<=rests]=0.
                forces=tensions[:,None]*directions
                apply_sensor(forces[0],nose)
                apply_aircraft(-forces[-1],anchor)
                fn[:k]+=forces[1:]-forces[:-1]
                # Distributed drag decomposed normal/tangential to the local cable.
                tangent=chain[2:]-chain[:-2]
                tangent/=np.maximum(np.linalg.norm(tangent,axis=1)[:,None],1e-12)
                vel=nodes[:k,3:]-self.wind(t)
                from .cable_flow import drag,nodal_exposure
                drag_args=(tangent,c['flight']['rho_kg_m3'],cable['diameter_m'],self.h,cable['cd_normal'],cable['cd_tangent'])
                if c['flight'].get('cable_bay_shielding',False):
                    exposed=np.ones(k) if self.phase=='deployed' else nodal_exposure(chain,a[:3],ra,c['bay']['exit_x_m'])
                    body_air=a[3:6]+cross(ra@a[10:13],nodes[:k,:3]-a[:3])
                    factor=c['flight']['internal_cable_flow_factor']
                    interior_air=body_air+factor*(self.wind(t)-body_air)
                    inside=nodes[:k,3:]-interior_air
                    external=vel*c['flight']['local_flow_factor']
                    internal=(1-exposed[:,None])*drag(inside,*drag_args)
                    fn[:k]+=exposed[:,None]*drag(external,*drag_args)+internal
                    # Air assumed to move with the enclosed bay transmits its
                    # drag reaction to the aircraft at the same cable points.
                    fa-=internal.sum(axis=0)
                    ta-=ra.T@cross(nodes[:k,:3]-a[:3],internal).sum(axis=0)
                    cable_exposure_mean=float(exposed.mean());internal_cable_drag=float(np.linalg.norm(internal,axis=1).sum())
                else:
                    fn[:k]+=drag(vel,*drag_args);cable_exposure_mean=1.
            support_power = -tensions[-1]*payout

        bay = c["bay"]
        local = ra.T@(s[:3]-a[:3])
        vlocal = ra.T@(s[3:6]-a[3:6])-cross(a[10:13], local)
        door_deg, door_rate_deg = self.door_state(t)
        door = np.deg2rad(door_deg)
        contact_total, penetration_max, latch_load, door_power = 0., 0., 0., 0.
        locked = self.phase == "stowed" or self.captured or (self.phase == "mission" and t < c["winch"]["release_s"])
        if self.sensor_enabled and self.phase != "deployed":
            if locked:
                target = a[:3]+ra@np.array(bay["stowed_center_m"])
                target_v = a[3:6]+ra@cross(a[10:13], bay["stowed_center_m"])
                f = bay["latch_k_N_m"]*(target-s[:3])+bay["latch_c_Ns_m"]*(target_v-s[3:6])
                apply_sensor(f, s[:3])
                apply_aircraft(-f, s[:3])
                if not self.point_mass:
                    error = attitude_error(self.latch_orientation(a), s[6:10])
                    torque_s = bay["latch_kr_Nm_rad"]*error-bay["latch_cr_Nms_rad"]*(s[10:13]-rs.T@ra@a[10:13])
                    ts += torque_s
                    ta -= ra.T@rs@torque_s
                latch_load = np.linalg.norm(f)
            # Contact samples include body ends and lateral/vertical fin tips.
            for p_sensor in ([] if self.mesh_contact_enabled or self.point_mass else c["sensor"]["contact_points_m"]):
                world, world_v = point_state(s, np.asarray(p_sensor))
                point = ra.T@(world-a[:3])
                rel = ra.T@(world_v-a[3:6])-cross(a[10:13], point)
                planes = []
                if point[0] < bay['exit_x_m']-bay['door_length_m']-.05 or point[0]>bay['front_x_m']+.05:
                    continue
                # Finite bay slabs give real edge/back-face contact on re-entry.
                # Truncating infinite half-spaces at the exit creates deep phantom
                # penetration as a sensor approaches from below the floor.
                back=bay['exit_x_m']-bay['lip_depth_m'];front=bay['front_x_m']
                middle=(back+front)/2;half_length=(front-back)/2
                thickness=bay['wall_thickness_m'];slope=bay['ramp_slope']
                scale=np.sqrt(1+slope*slope)
                floor_down=np.array([slope,0.,1.])/scale
                floor_axes=np.column_stack([[1/scale,0,-slope/scale],[0,1,0],floor_down])
                floor_center=np.array([middle,0,bay['floor_z_m']+slope*(front-middle)])+floor_down*thickness/2
                floor_low=bay['floor_z_m']+slope*(front-back)
                zmid=(bay['ceiling_z_m']+floor_low)/2;zhalf=(floor_low-bay['ceiling_z_m'])/2
                boxes=[(floor_center,floor_axes,[half_length*scale,bay['half_width_m'],thickness/2]),
                       ([middle,0,bay['ceiling_z_m']-thickness/2],np.eye(3),[half_length,bay['half_width_m'],thickness/2])]
                for sign in [-1,1]:
                    boxes.append(([middle,sign*(bay['half_width_m']+thickness/2),zmid],np.eye(3),[half_length,thickness/2,zhalf]))
                for center,axes,half in boxes:
                    normal,penetration=sphere_box_contact(point,center,axes,half,bay['contact_radius_m'])
                    planes.append((normal,penetration,np.zeros(3)))
                # Hinged rear ramp: finite plate, x extends aft as it opens down.
                from .door import door_hinge, door_box
                hinge = door_hinge(bay)
                delta = point-hinge
                center,axes,half=door_box(bay,door_deg)
                contact_normal,penetration=sphere_box_contact(point,center,axes,half,bay['contact_radius_m'])
                door_velocity = cross([0., np.deg2rad(door_rate_deg), 0.], delta)
                planes.append((contact_normal, penetration, door_velocity))
                for normal, penetration, surface_velocity in planes:
                    if penetration <= 0:
                        continue
                    contact_velocity = rel-surface_velocity
                    normal_speed = np.dot(contact_velocity, normal)
                    normal_f, friction_f = contact_force(penetration, normal_speed, contact_velocity-normal_speed*normal,
                                                       bay["contact_k_N_m"], bay["contact_c_Ns_m"], bay["friction"])
                    force_b = normal_f*normal+friction_f
                    apply_sensor(ra@force_b, world)
                    apply_aircraft(-ra@force_b, world)
                    contact_total += normal_f
                    door_power += np.dot(force_b, surface_velocity)
                    penetration_max = max(penetration_max, penetration)
            if not locked and bay["release_push_N"] and t < bay.get('release_push_until_s', float('inf')):
                f = ra@np.array([-bay["release_push_N"] if local[0]>bay["exit_x_m"] else 0., 0, 0])
                apply_sensor(f, s[:3]); apply_aircraft(-f, s[:3])

        cable_contact, min_gap, mesh_contacts = 0., np.nan, 0
        if self.mesh_contact_enabled:
            from .collision import barrier_force
            cc = c['collision']
            hits = self.contact_geometry(t,y)
            min_gap = min((h.get('geometry_gap',h['gap']) for h in hits),default=cc['skin_m'])
            chain = np.vstack([nose,nodes[:k,:3],anchor])
            chain_v = np.vstack([nose_v,nodes[:k,3:],anchor_v])
            from .contact_batch import sensor_contact_loads
            batch=sensor_contact_loads(hits,a,s,ra,rs,cc['skin_m'],cc.get('part_materials',{}),cc,bay['friction'],door_rate_deg,self._mesh_contacts.hinge)
            if batch is not None:
                fs+=batch['sensor_force'];ts+=batch['sensor_torque'];fa+=batch['aircraft_force'];ta+=batch['aircraft_torque']
                contact_total+=batch['contact_N'];mesh_contacts+=batch['count'];door_power+=batch['door_power_W']
                penetration_max=max(penetration_max,batch['penetration_m'])
            for hit in hits:
                if hit['kind']=='sensor':continue
                gap, normal = hit['gap'], hit['normal']
                if gap >= cc['skin_m']:
                    continue
                point = hit['point']
                if hit['kind']=='sensor':
                    world = a[:3]+ra@point
                    velocity = s[3:6]+cross(rs@s[10:13],world-s[:3])
                else:
                    j=hit['span'];delta=chain[j+1]-chain[j]
                    fraction=float(np.clip(np.dot(a[:3]+ra@point-chain[j],delta)/max(np.dot(delta,delta),1e-20),0,1))
                    world=(1-fraction)*chain[j]+fraction*chain[j+1]
                    velocity=(1-fraction)*chain_v[j]+fraction*chain_v[j+1]
                    point=ra.T@(world-a[:3])
                surface_v = cross([0.,np.deg2rad(door_rate_deg),0.],point-self._mesh_contacts.hinge) if hit['door'] else np.zeros(3)
                rel = ra.T@(velocity-a[3:6])-cross(a[10:13],point)-surface_v
                speed = float(np.dot(rel,normal))
                material=cc.get('part_materials',{}).get(hit['fixed'],cc)
                magnitude,friction=barrier_force(gap,speed*hit.get('normal_velocity_scale',1.),rel-speed*normal,cc['skin_m'],
                    material['stiffness_N_m'],material['damping_Ns_m'],bay['friction'])
                weight=hit.get('weight',1.)
                magnitude*=weight;friction*=weight
                force_b=magnitude*normal+friction;force=ra@force_b
                if hit['kind']=='sensor':
                    apply_sensor(force,world)
                else:
                    # Linear shape functions map the segment contact to both
                    # material nodes. The reaction acts at the SAME interpolated
                    # point, preserving linear and angular momentum exactly.
                    for end,weight in ((j,1-fraction),(j+1,fraction)):
                        if end==0:apply_sensor(weight*force,nose)
                        elif end==k+1:apply_aircraft(weight*force,anchor)
                        else:fn[end-1]+=weight*force
                    cable_contact+=magnitude
                apply_aircraft(-force,world)
                contact_total+=magnitude;mesh_contacts+=1
                penetration_max=max(penetration_max,max(0.,-gap))
                door_power+=np.dot(force_b,surface_v)

        out = np.zeros_like(y)
        for off, b, force, torque, mass, inertia, inverse in ((0, a, fa, ta, self.ma, self.Ia, self.Iai), (13, s, fs, ts, self.ms, self.Is, self.Isi)):
            out[off:off+3] = b[3:6]
            out[off+3:off+6] = force/mass
            if off == 13 and self.point_mass:
                continue  # Retained state slots are inert, not rotational DOFs.
            out[off+6:off+10] = qdot(b[6:10], b[10:13])
            out[off+10:off+13] = inverse@(torque-cross(b[10:13], inertia@b[10:13]))
        out[26:].reshape(self.n, 6)[:, :3] = nodes[:, 3:]
        out[26:].reshape(self.n, 6)[:, 3:] = fn/self.mn
        if not self.sensor_enabled:
            out[13:] = 0.
        if diagnostics:
            tension = max(tensions, default=0.)
            reel_tension = tensions[-1] if len(tensions) else 0.
            d.update({"length_m": length, "payout_m_s": payout, "tension_N": tension, "reel_tension_N": reel_tension,
                      "torque_Nm": reel_tension*c["winch"]["radius_m"], "winch_power_W": support_power,
                      "contact_N": contact_total, "latch_N": latch_load, "capture_N": latch_load if self.captured else 0.,
                      "door_power_W": door_power, "penetration_m": penetration_max, "active_nodes": k,
                      "cable_contact_N": cable_contact, "minimum_mesh_gap_m": min_gap,
                      "cable_exposure_mean":cable_exposure_mean,"internal_cable_drag_N":internal_cable_drag,
                      "mesh_contact_count": mesh_contacts,
                      "sensor_exposure": exposure, "door_deg": door_deg,
                      "door_command_deg": schedule(c['winch']['door_schedule'],t)[0],
                      "door_interlock_active": self.door_closing_start() is not None and t >= self.door_closing_start() and (not self.captured or t < max(self.door_closing_start(), self.capture_time+c['winch'].get('door_capture_hold_s',0.))),
                      "sensor_local_x_m": local[0], "sensor_local_y_m": local[1], "sensor_local_z_m": local[2],
                      "sensor_relative_speed_m_s": np.linalg.norm(vlocal), "captured": self.captured,
                      "total_mass_kg": self.ma+(self.ms+self.mn*self.n if self.sensor_enabled else 0.),
                      "sensor_accel_m_s2": np.linalg.norm(fs/self.ms),
                      "phase": "aircraft_only" if not self.sensor_enabled else "captured" if self.captured else "stowed" if locked else "internal" if exposure<1 else "payout" if payout>0 else "recovery" if payout<0 else "tow"})
            return out, d
        return out

    def capture_metric(self, t, y):
        a, s = y[:13], y[13:26]
        r = rotation(a[6:10])
        local = r.T@(s[:3]-a[:3])
        relative = r.T@(s[3:6]-a[3:6])-cross(a[10:13], local)
        bay = self.c["bay"]
        return max(np.linalg.norm(local-np.array(bay["stowed_center_m"]))/bay["capture_radius_m"],
                   np.linalg.norm(relative)/bay["capture_speed_m_s"],
                   0. if self.point_mass else np.linalg.norm(attitude_error(self.latch_orientation(a), s[6:10]))/np.deg2rad(bay["capture_angle_deg"]))-1

    def jac_sparsity(self):
        from scipy.sparse import lil_matrix
        m = lil_matrix((self.size, self.size), dtype=int)
        m[:26, :26] = 1
        k=self.n if self.active_override is None else self.active_override
        if k:
            m[:13,26+6*(k-1):26+6*k]=1
            m[13:26,26:32]=1
            if self.mesh_contact_enabled or self.c['flight'].get('cable_bay_shielding',False):
                # Any exposed cable cell can touch any aircraft surface.
                m[:13,26:26+6*k]=1
        m[:13,26+6*k:]=1
        for j in range(self.n):
            row = slice(26+6*j, 26+6*(j+1))
            m[row, :26] = 1
            m[row, 26+6*max(0, j-1):26+6*min(self.n, j+2)] = 1
        return m.tocsr()

    def jacobian(self):
        """Grouped finite differences plus exact stored-cell derivatives.

        Stored cells all react on the aircraft; treating those columns as dense
        finite differences makes mesh refinement needlessly expensive.
        """
        from scipy.sparse import csc_matrix
        from .math3d import skew
        k=self.n if self.active_override is None else self.active_override
        pattern=self.jac_sparsity().toarray().astype(bool)
        columns=26+6*k
        groups=[]; occupied=[]
        for j in range(columns):
            for idx, used in enumerate(occupied):
                if not np.any(used & pattern[:,j]):
                    groups[idx].append(j);occupied[idx]|=pattern[:,j];break
            else:
                groups.append([j]);occupied.append(pattern[:,j].copy())
        def evaluate(t,y):
            base=self.rhs(t,y);matrix=np.zeros((self.size,self.size))
            step=np.sqrt(np.finfo(float).eps)*np.maximum(1.,np.abs(y))*self.c.get('simulation',{}).get('jacobian_step_factor',1.)
            perturbed=np.repeat(y[:,None],len(groups),axis=1)
            for index,group in enumerate(groups):
                perturbed[group,index]+=step[group]
            differences=self.rhs_batch(t,perturbed)-base[:,None]
            for index,group in enumerate(groups):
                delta=differences[:,index]
                for j in group:
                    rows=pattern[:,j];matrix[rows,j]=delta[rows]/step[j]
            a=y[:13];ra=rotation(a[6:10]);anchor,velocity=point_state(a,self.attach)
            stiffness=self.c['cable']['spool_k_N_m']/self.n
            damping=self.c['cable']['spool_c_Ns_m']/self.n
            for j in range(k,self.n):
                off=26+6*j;p=y[off:off+3];v=y[off+3:off+6]
                f=-stiffness*(p-anchor)-damping*(v-velocity)
                matrix[3:6,off:off+3]=np.eye(3)*stiffness/self.ma
                matrix[3:6,off+3:off+6]=np.eye(3)*damping/self.ma
                matrix[10:13,off:off+3]=self.Iai@ra.T@(skew(f)+stiffness*skew(p-a[:3]))
                matrix[10:13,off+3:off+6]=self.Iai@ra.T@(damping*skew(p-a[:3]))
                matrix[off:off+3,off+3:off+6]=np.eye(3)
                matrix[off+3:off+6,off:off+3]=-np.eye(3)*stiffness/self.mn
                matrix[off+3:off+6,off+3:off+6]=-np.eye(3)*damping/self.mn
            if not self.sensor_enabled:
                matrix[13:,:]=0;matrix[:,13:]=0
            return csc_matrix(matrix)
        return evaluate

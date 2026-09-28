from pathlib import Path
import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots
from .math3d import rotation
from .door import door_box


def validity_annotation(fig, result):
    if result.summary.get('geometry_validity') == 'invalid':
        fig.add_annotation(
            x=0.5,
            y=1.12,
            xref='paper',
            yref='paper',
            showarrow=False,
            text='INVALID TRAJECTORY: CAD intersections detected. Loads are not valid design results.',
            font=dict(color='#b42318', size=14),
            bgcolor='#fff0ec',
        )
    return fig


def history_figure(result):
    t = result.table
    controlled = result.summary.get('controller_enabled', False)
    rows = 6 if controlled else 3
    titles = ('Aircraft attitude', 'Cable and contact loads', 'Sensor position relative to aircraft')
    fig = make_subplots(
        rows=rows,
        cols=1,
        shared_xaxes=True,
        subplot_titles=titles + ('Altitude tracking', 'Airspeed tracking', 'Elevator and thrust commands')
        if controlled
        else titles,
        specs=[[{}] for _ in range(rows - 1)] + [[{'secondary_y': True} if controlled else {}]],
    )
    for key, color in [('roll_deg', '#2563a6'), ('pitch_deg', '#c57a22'), ('yaw_deg', '#63764c')]:
        fig.add_trace(go.Scatter(x=t.time_s, y=t[key], name=key, line=dict(color=color)), row=1, col=1)
    for key, color in [('tension_N', '#2563a6'), ('contact_N', '#c57a22'), ('capture_N', '#63764c')]:
        fig.add_trace(go.Scatter(x=t.time_s, y=t[key], name=key, line=dict(color=color)), row=2, col=1)
    for key, color in [
        ('sensor_local_x_m', '#2563a6'),
        ('sensor_local_y_m', '#c57a22'),
        ('sensor_local_z_m', '#63764c'),
    ]:
        fig.add_trace(go.Scatter(x=t.time_s, y=t[key], name=key, line=dict(color=color)), row=3, col=1)
    fig.update_yaxes(title_text='Angle (deg)', row=1, col=1)
    fig.update_yaxes(title_text='Force (N)', row=2, col=1)
    fig.update_yaxes(title_text='Position (m)', row=3, col=1)
    if controlled:
        for row, actual, target, unit in [
            (4, 'altitude_m', 'altitude_setpoint_m', 'Altitude (m)'),
            (5, 'airspeed_m_s', 'airspeed_setpoint_m_s', 'Airspeed (m/s)'),
        ]:
            fig.add_trace(go.Scatter(x=t.time_s, y=t[actual], name=actual), row=row, col=1)
            fig.add_trace(go.Scatter(x=t.time_s, y=t[target], name=target, line=dict(dash='dash')), row=row, col=1)
            fig.update_yaxes(title_text=unit, row=row, col=1)
        fig.add_trace(go.Scatter(x=t.time_s, y=t.elevator_deg, name='elevator_deg'), row=6, col=1, secondary_y=False)
        fig.add_trace(go.Scatter(x=t.time_s, y=t.thrust_N, name='thrust_N'), row=6, col=1, secondary_y=True)
        fig.update_yaxes(title_text='Elevator (deg)', row=6, col=1, secondary_y=False)
        fig.update_yaxes(title_text='Thrust (N)', row=6, col=1, secondary_y=True)
    fig.update_xaxes(title_text='Time (s)', row=rows, col=1)
    fig.update_layout(
        template='plotly_white',
        height=1500 if controlled else 850,
        title=result.summary['aero_metadata']['solver'] + ' coupled case — unvalidated assumptions',
        legend=dict(orientation='h'),
        margin=dict(t=100, b=90),
    )
    return validity_annotation(fig, result)


def replay_figure(result):
    cfg = result.config
    n = cfg['cable']['segments']
    indices = np.linspace(0, len(result.time) - 1, min(180, len(result.time))).astype(int)

    def traces(idx):
        y = result.states[idx]
        r = rotation(y[6:10])
        rs = rotation(y[19:23])
        origin = y[:3]

        def local(p):
            return (np.asarray(p) - origin) @ r

        sensor = local(y[13:16])
        nodes = local(y[26:].reshape(n, 6)[:, :3])
        anchor = np.array(cfg['aircraft']['tow_point_m'])
        k = int(result.table.active_nodes.iloc[idx])
        nose = local(y[13:16] + rs @ np.array(cfg['sensor']['tow_point_m']))
        cable = np.vstack([nose, nodes[:k], anchor])
        body = np.array([[0.3, 0, 0], [-0.55, 0, 0]])
        wing = np.array([[0, -0.85, 0], [0, 0.85, 0]])
        half = 0.0 if cfg['sensor'].get('model') == 'point_mass' else cfg['sensor']['length_m'] / 2
        sensor_line = local(np.array([y[13:16] + rs @ np.array([x, 0, 0]) for x in [-half, half]]))

        b = cfg['bay']
        center, axes, halfbox = door_box(b, result.table.door_deg.iloc[idx])
        hinge = center + axes @ np.array([0.0, 0.0, halfbox[2]])
        tip = center - axes @ np.array([0.0, 0.0, halfbox[2]])
        data = []
        exit_ring = np.array(
            [
                [b['exit_x_m'], side, z]
                for side, z in [
                    (-b['half_width_m'], b['ceiling_z_m']),
                    (b['half_width_m'], b['ceiling_z_m']),
                    (b['half_width_m'], b['floor_z_m']),
                    (-b['half_width_m'], b['floor_z_m']),
                    (-b['half_width_m'], b['ceiling_z_m']),
                ]
            ]
        )
        for p, name, color, width in [
            (body, 'Aircraft', '#404040', 7),
            (wing, 'Wing', '#404040', 7),
            (cable, 'Cable', '#2563a6', 4),
            (sensor_line, 'Sensor', '#c57a22', 9),
            (np.vstack([hinge, tip]), 'Door', '#63764c', 7),
            (exit_ring, 'Exit', '#7a6a86', 4),
        ]:
            data.append(
                go.Scatter3d(
                    x=p[:, 0],
                    y=p[:, 1],
                    z=-p[:, 2],
                    mode='lines+markers',
                    name=name,
                    line=dict(color=color, width=width),
                    marker=dict(size=2),
                )
            )
        return data

    frames = [go.Frame(data=traces(i), name=str(j)) for j, i in enumerate(indices)]
    fig = go.Figure(data=traces(indices[0]), frames=frames)
    L = cfg['cable']['length_m']
    fig.update_layout(
        title='Body-relative replay — z displayed upward (m)',
        template='plotly_white',
        height=650,
        scene=dict(
            xaxis=dict(range=[-L - 1, 1], title='Forward x (m)'),
            yaxis=dict(range=[-L / 2, L / 2], title='Right y (m)'),
            zaxis=dict(range=[-L, L / 2], title='Up (m)'),
            aspectmode='data',
        ),
        updatemenus=[
            dict(
                type='buttons',
                buttons=[
                    dict(
                        label='Play',
                        method='animate',
                        args=[None, dict(frame=dict(duration=70, redraw=True), fromcurrent=True)],
                    ),
                    dict(
                        label='Pause',
                        method='animate',
                        args=[[None], dict(mode='immediate', frame=dict(duration=0, redraw=False))],
                    ),
                ],
            )
        ],
        sliders=[
            dict(
                steps=[
                    dict(
                        method='animate',
                        args=[[str(j)], dict(mode='immediate', frame=dict(duration=0, redraw=True))],
                        label=f'{result.time[i]:.2f}s',
                    )
                    for j, i in enumerate(indices)
                ]
            )
        ],
    )
    return validity_annotation(fig, result)


def export_plots(result, directory):
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    history_figure(result).write_html(directory / 'history.html', include_plotlyjs=True)
    replay_figure(result).write_html(directory / 'replay.html', include_plotlyjs=True, auto_play=False)

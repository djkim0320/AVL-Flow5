// Older saved projects retain their physical inputs but use the two current scenarios.
export function currentScenario(settings) {
  const task=['mission','recovery'].includes(settings.task)?'recovery':'flight';
  return {...settings,task,phase:task==='recovery'?'recovery':'deployed',start:'equilibrium'};
}

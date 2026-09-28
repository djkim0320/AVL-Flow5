// Old editor drafts now open in the requested continuous sequence. Explicit
// standalone choices saved by this UI version remain available for diagnostics.
export function currentScenario(settings) {
  const task=settings.sequence_ui_version===1&&['sequence','flight','recovery'].includes(settings.task)?settings.task:'sequence';
  const migrated={...settings,sequence_ui_version:1,task,phase:task==='sequence'?'stowed':task==='recovery'?'recovery':'deployed',start:'equilibrium'};
  // Blank fields from the old, inactive UI must not erase visible defaults.
  if(settings.sequence_ui_version!==1)for(const key of ['preflight','payout','hold','recovery','recovery_length'])if(migrated[key]==null)delete migrated[key];
  return migrated;
}

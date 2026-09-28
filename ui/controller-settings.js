// Old projects had only a disabled switch. Preserve all explicit tuning.
export function restoreController(value, defaults){
  return value && typeof value==='object' && !Array.isArray(value) &&
    Object.keys(value).every(key=>key==='enabled') ? structuredClone(defaults) : structuredClone(value);
}

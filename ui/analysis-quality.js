// Execution status, linear modes and validation are separate statements.
export function resultQuality(result = {}) {
  const status = result.stability_status;
  let stability = null;
  if (status === 'unstable_detected' || result.unstable === true) stability = '불안정 모드 검출';
  else if (status === 'near_neutral') stability = '판정 보류 · 0 근처 모드';
  else if (status === 'damped') stability = '계산 모드는 감쇠 · 실물 검증 전';
  else if (result.unstable === false) stability = '기존 결과 · 안정성 추가 확인 필요';
  return {
    stability,
    convergence: result.numerically_converged === true ? '수렴 확인 · 해당 검사 범위' : '수치 수렴 미확인',
    note: '계산 완료와 검증 완료는 다릅니다. 고유값은 고정 길이 평형의 결과이며, 회수 중 안정성을 보증하지 않습니다.'
  };
}

export function hybridQuality(meta) {
  if (!meta || meta.backend !== 'hybrid') return null;
  const labels = {
      CL_alpha_per_deg: 'CLα',
      Cm_alpha_per_deg: 'Cmα',
      CY_beta_per_deg: 'CYβ',
      Cl_beta_per_deg: 'Clβ',
      Cn_beta_per_deg: 'Cnβ'
    },
    c = meta.composition,
    rows = Object.entries(meta.consistency || {}).map(([name, r]) => ({
      name: labels[name] || name,
      avl: r.avl,
      flow5: r.flow5,
      difference:
        r.relative_difference == null ? '두 기울기 모두 0 근처' : `${(100 * r.relative_difference).toFixed(1)}%`,
      warning: r.warning
    }));
  return {
    source: `정적: ${c.coeff} · 조종: ${c.controls} · 회전율: ${c.rates} · flow5 ${meta.flow5_speed_m_s} m/s`,
    rows,
    note: rows.some(r => r.warning)
      ? '두 해석기의 기울기 차이가 20%를 넘는 항목이 있습니다. 정확도 판정이 아니라 모델 차이 점검입니다.'
      : '두 해석기의 기울기를 비교했습니다. 값의 일치가 정확도를 입증하지는 않습니다.'
  };
}

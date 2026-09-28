import numpy as np
import pytest
from dbf_stability.stability_quality import describe_spectrum


def test_nearly_undamped_oscillation_is_not_called_stable():
    result=describe_spectrum([7.6e-6+2j,7.6e-6-2j,-.4,0])
    assert result['stability_status']=='near_neutral'
    assert result['near_neutral_modes']==3
    assert set(result['modes'].classification)=={'stable','near_neutral'}
    assert result['linearization_verified'] is False


def test_growth_is_reported_even_with_free_translation_roots():
    result=describe_spectrum([.00672,0,0,-2+3j,-2-3j])
    assert result['unstable']
    assert result['stability_status']=='unstable_detected'
    assert result['growth_time_s']==pytest.approx(1/.00672)


@pytest.mark.parametrize('eigenvalues',[[],[np.nan],[np.inf],[[1,2]]])
def test_invalid_spectra_are_rejected(eigenvalues):
    with pytest.raises(ValueError):describe_spectrum(eigenvalues)

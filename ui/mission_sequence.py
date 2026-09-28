"""Named phases shared by CSV and replay; boundaries come from the solver input."""


def stage_at(time_s, schedule):
    if time_s < schedule['release_s']:return '비행 시작'
    if time_s < schedule['deployed_s']:return '전개'
    if time_s < schedule['recovery_s']:return '전개 후 비행'
    return '회수'

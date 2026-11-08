const labels = new Map(Object.entries({
  FOLLOW_OBSERVE: '前車に追従中',
  OPPORTUNITY_READY: '追い越し条件を確認中',
  PASS: '追い越し中',
  AVOID_STATIC: '停止車両を回避中',
  BLOCKED: '回避経路なし・停止目標',
  STATIC_CLEAR: '停止車両は走路外',
  MERGE: '合流中',
  PASSED: '合流完了',
  FAULT: '計画無効',
}));

export const racingLabel = state => labels.get(state) ?? '状態不明';

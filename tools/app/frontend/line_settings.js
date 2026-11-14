import { $, state } from './api.js';

export function restoreLineSettings(lane) {
  Object.assign(state.settings, lane.lines.centerline?.settings || {},
    lane.lines.customline?.settings || {}, lane.lines.raceline?.settings || {});
  state.settings.optimizer = lane.lines.raceline?.settings?.optimizer || 'mincurv';
  state.settings.centerline_method = lane.lines.centerline?.settings?.centerline_method || 'region';
  state.settings.region_resolution = lane.lines.centerline?.settings?.region_resolution ?? .02;
  state.settings.centerline_max_points = lane.lines.centerline?.settings?.centerline_max_points ?? 800;
  $('raceline-method').value = state.settings.optimizer;
  $('centerline-method').value = state.settings.centerline_method;
}

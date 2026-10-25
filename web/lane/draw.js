// Drawing helpers for the lane overlay, shared by web/replay.html.
// Copied from web/spike-ground.html (same functions, now exported) so the replay overlay looks
// exactly like the live app. If the live app changes its drawing, update this copy too
// (or switch spike-ground.html to import from here).

import { groundToPixel } from './ground.js';
import { LaneStatus } from './tracker.js';
import { Pattern } from '../control/controller.js';

// Point on the line at perpendicular offset y, distance t along the lane heading.
export const alongLane = (t, y, heading) => [
  t * Math.cos(heading) - y * Math.sin(heading),
  t * Math.sin(heading) + y * Math.cos(heading),
];

// A Lock preview in the same shape the tracker outputs, for drawing.
export function candidateLane(cand, heading) {
  return { status: 'candidate', heading, width: cand.width, polylines: cand.polylines };
}

// Big text over the camera view, like the reference's "Straight".
export function cueText(cmd, locked, found) {
  if (!locked) return found ? 'LANE FOUND' : 'NO LANE';
  if (cmd.pattern === Pattern.STOP) return 'STOP';
  if (cmd.pattern === Pattern.HEARTBEAT) return 'STRAIGHT';
  if (cmd.pattern === Pattern.CURVE) return cmd.motors & 0b00011 ? '◀ CURVE' : 'CURVE ▶';
  if (cmd.pattern === Pattern.STEER) return cmd.motors & 0b00011 ? '◀ LEFT' : 'RIGHT ▶';
  return '';
}

// The lane as found segment by segment: each line is the chain of points the sliding window
// matched (joined short straight pieces), and the space between the two lines is filled, as in
// the ideal case. A side with no chain is drawn one lane width from the other side.
export function drawLane(ctx, toScreen, lane) {
  if (!lane || !lane.polylines) return;
  let { left, right } = lane.polylines;
  if (!left.length && !right.length) return;
  if (!left.length) left = right.map((p) => ({ x: p.x, y: p.y - lane.width }));
  if (!right.length) right = left.map((p) => ({ x: p.x, y: p.y + lane.width }));
  // Reach toward the runner's feet and a little past the last segment, along each line's own end
  // direction, so the fill covers the whole lane in view like the ideal case.
  const extend = (chain) => {
    if (chain.length < 2) return chain;
    const [a0, a1] = chain;
    const [z1, z0] = chain.slice(-2);
    const s0 = (a1.y - a0.y) / (a1.x - a0.x);
    const s1 = (z0.y - z1.y) / (z0.x - z1.x);
    return [{ x: 0.5, y: a0.y - s0 * (a0.x - 0.5) }, ...chain, { x: z0.x + 0.5, y: z0.y + s1 * 0.5 }];
  };
  const toPts = (chain) => extend(chain).map((p) => toScreen(...alongLane(p.x, p.y, lane.heading))).filter(Boolean);
  const l = toPts(left);
  const r = toPts(right);
  const preview = lane.status === 'candidate';
  const outside = lane.status === LaneStatus.OUTSIDE_LEFT || lane.status === LaneStatus.OUTSIDE_RIGHT;
  if (l.length && r.length) {
    ctx.fillStyle = preview ? 'rgba(0, 230, 30, 0.15)' : outside ? 'rgba(255, 60, 60, 0.35)' : 'rgba(0, 230, 30, 0.35)';
    ctx.beginPath();
    [...l, ...[...r].reverse()].forEach(([x, y], i) => (i ? ctx.lineTo(x, y) : ctx.moveTo(x, y)));
    ctx.closePath();
    ctx.fill();
  }
  ctx.strokeStyle = '#ff3';
  ctx.lineWidth = preview ? 2 : 5;
  ctx.setLineDash(preview ? [6, 6] : []);
  for (const pts of [l, r]) {
    ctx.beginPath();
    pts.forEach(([x, y], i) => (i ? ctx.lineTo(x, y) : ctx.moveTo(x, y)));
    ctx.stroke();
  }
  ctx.setLineDash([]);

  // The reference line the PD controller follows: the fitted centerline (eY and eψ are measured
  // against it at the runner). Before Lock, the middle of the two found lines.
  const xs = [];
  for (let x = 0.5; x <= Math.max(...left.map((p) => p.x), ...right.map((p) => p.x)) + 0.5; x += 0.25) xs.push(x);
  const { shape } = lane;
  const centerY = shape
    ? (x) => shape.a + shape.b * x + shape.c * x * x
    : (x) => (interp(left, x) + interp(right, x)) / 2;
  const center = xs.map((x) => toScreen(...alongLane(x, centerY(x), lane.heading))).filter(Boolean);
  ctx.strokeStyle = '#fff';
  ctx.lineWidth = preview ? 2 : 3;
  ctx.setLineDash([10, 8]);
  ctx.beginPath();
  center.forEach(([x, y], i) => (i ? ctx.lineTo(x, y) : ctx.moveTo(x, y)));
  ctx.stroke();
  ctx.setLineDash([]);
  if (preview) return;

  // The runner's own straight-ahead path (camera ground frame, y = 0). Its gap to the centerline is
  // the error the belt is correcting.
  const path = [0.3, 1, 2, 3].map((x) => toScreen(x, 0)).filter(Boolean);
  ctx.strokeStyle = '#3cf';
  ctx.lineWidth = 3;
  ctx.beginPath();
  path.forEach(([x, y], i) => (i ? ctx.lineTo(x, y) : ctx.moveTo(x, y)));
  ctx.stroke();
}

// Linear interpolation / end extrapolation of a chain's y at distance x.
export function interp(chain, x) {
  if (chain.length === 1) return chain[0].y;
  let i = 1;
  while (i < chain.length - 1 && chain[i].x < x) i++;
  const a = chain[i - 1];
  const b = chain[i];
  return a.y + ((b.y - a.y) * (x - a.x)) / (b.x - a.x);
}

export function drawView(ctx, img, detector, res, lane, cue, aim) {
  const s = 2;
  const tmp = new OffscreenCanvas(img.width, img.height);
  const tctx = tmp.getContext('2d');
  const mask = detector.mask.data;
  for (let i = 0; i < mask.length; i++) {
    if (mask[i]) {
      img.data[4 * i] = 255;
      img.data[4 * i + 1] = 40;
      img.data[4 * i + 2] = 40;
    }
  }
  tctx.putImageData(img, 0, 0);
  ctx.imageSmoothingEnabled = false;
  ctx.drawImage(tmp, 0, 0, img.width * s, img.height * s);
  if (res.ok) {
    drawLane(ctx, (x, y) => {
      const px = groundToPixel(x, y, detector.K, res.basis, detector.cfg.cameraHeight);
      return px && [px[0] * s, px[1] * s];
    }, lane);
    // Pure-pursuit aim point (orange): the belt steers the runner toward it.
    const a = aim && groundToPixel(aim.x, aim.y, detector.K, res.basis, detector.cfg.cameraHeight);
    if (a) {
      ctx.fillStyle = '#f80';
      ctx.strokeStyle = '#000';
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.arc(a[0] * s, a[1] * s, 9, 0, 2 * Math.PI);
      ctx.fill();
      ctx.stroke();
    }
  }
  if (cue) {
    ctx.font = `bold ${Math.round(ctx.canvas.width / 9)}px -apple-system, system-ui, sans-serif`;
    ctx.textAlign = 'center';
    ctx.lineWidth = 6;
    ctx.strokeStyle = 'rgba(0, 0, 0, 0.6)';
    ctx.fillStyle = cue === 'STOP' || cue === 'NO LANE' ? '#f55' : '#fff';
    ctx.strokeText(cue, ctx.canvas.width / 2, ctx.canvas.height * 0.12);
    ctx.fillText(cue, ctx.canvas.width / 2, ctx.canvas.height * 0.12);
  }
}

// Bird's-eye view: forward 0..5m bottom to top, lateral -3..3m. Red dots: line pixels on the
// ground; white ticks: line peaks found per segment; the lane as in drawLane.
export function drawBev(canvas, res, lane) {
  const ctx = canvas.getContext('2d');
  const W = canvas.width;
  const H = canvas.height;
  const sx = (y) => W / 2 + (y / 3) * (W / 2);
  const sy = (x) => H - (x / 5) * H;
  ctx.fillStyle = '#000';
  ctx.fillRect(0, 0, W, H);
  ctx.fillStyle = '#333';
  ctx.fillRect(0, sy(4), W, sy(1) - sy(4));
  ctx.fillStyle = '#fff';
  ctx.beginPath();
  ctx.arc(sx(0), sy(0) - 4, 4, 0, 2 * Math.PI);
  ctx.fill();
  drawLane(ctx, (x, y) => [sx(y), sy(x)], lane);
  if (!res.ok) return;
  ctx.fillStyle = '#f55';
  const { points, count } = res.ground;
  for (let i = 0; i < count; i++) ctx.fillRect(sx(points[2 * i + 1]), sy(points[2 * i]), 1.5, 1.5);
  ctx.strokeStyle = '#fff';
  ctx.lineWidth = 2;
  for (const seg of res.segments) {
    for (const p of seg.peaks) {
      const [x0, y0] = alongLane(seg.x - 0.15, p.y, res.heading);
      const [x1, y1] = alongLane(seg.x + 0.15, p.y, res.heading);
      ctx.beginPath();
      ctx.moveTo(sx(y0), sy(x0));
      ctx.lineTo(sx(y1), sy(x1));
      ctx.stroke();
    }
  }
}

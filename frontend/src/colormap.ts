/** Google's "turbo" colormap (polynomial fit), the CFD-style rainbow from blue (0) to red (1). */
export function turbo(x: number): [number, number, number] {
  const t = Math.min(1, Math.max(0, x));
  const r = 0.13572138 + t * (4.6153926 + t * (-42.66032258 + t * (132.13108234 + t * (-152.94239396 + t * 59.28637943))));
  const g = 0.09140261 + t * (2.19418839 + t * (4.84296658 + t * (-14.18503333 + t * (4.27729857 + t * 2.82956604))));
  const b = 0.1066733 + t * (12.64194608 + t * (-60.58204836 + t * (110.36276771 + t * (-89.90310912 + t * 27.34824973))));
  const c = (v: number) => Math.min(1, Math.max(0, v));
  return [c(r), c(g), c(b)];
}

export function turboCss(x: number): string {
  const [r, g, b] = turbo(x).map((v) => Math.round(v * 255));
  return `rgb(${r}, ${g}, ${b})`;
}

/** CSS gradient of the turbo map for a legend bar. */
export function turboGradient(steps = 12): string {
  const stops = Array.from({ length: steps + 1 }, (_, i) => `${turboCss(i / steps)} ${((100 * i) / steps).toFixed(1)}%`);
  return `linear-gradient(to right, ${stops.join(", ")})`;
}

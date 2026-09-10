/** Fixed-point number formatting for the metrics panel; non-finite values render as "-". */
export function fmt(value: number | undefined | null, digits = 2): string {
  if (value === undefined || value === null || !Number.isFinite(value)) return "-";
  return value.toFixed(digits);
}

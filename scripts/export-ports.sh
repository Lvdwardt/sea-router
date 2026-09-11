#!/usr/bin/env bash
# Refresh rust/tests/fixtures/ports.json from the cruisello database.
#
# The audit and its tests run against a frozen copy of the port list so they
# need no database. Re-run this when ports are added or their coordinates are
# corrected, then regenerate the baseline:
#
#   scripts/export-ports.sh
#   cd rust && ./target/release/sea-router-rs audit --write-baseline
#
# Requires DATABASE_URL (or pass a connection string as $1).
set -euo pipefail

DB="${1:-${DATABASE_URL:-}}"
if [ -z "$DB" ]; then
  echo "usage: DATABASE_URL=... $0   (or: $0 <connection-string>)" >&2
  exit 1
fi

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT="$ROOT/rust/tests/fixtures/ports.json"

psql "$DB" -t -A -c "
select json_agg(row_to_json(t) order by t.name)::text
from (
  select p.name,
         coalesce(p.country_code,'') as country,
         round(p.latitude::numeric, 6)::float8 as lat,
         round(p.longitude::numeric, 6)::float8 as lon,
         (select count(*) from itinerary_days d where d.port_id = p.id)::int as days
  from ports p
  where p.latitude is not null and p.longitude is not null
) t
" | python3 -c "
import json, sys
ports = json.load(sys.stdin)
out = ['[']
for i, p in enumerate(ports):
    out.append('  ' + json.dumps(p, ensure_ascii=False, sort_keys=True) + (',' if i < len(ports)-1 else ''))
out.append(']')
open('$OUT','w').write('\n'.join(out) + '\n')
print(f'{len(ports)} ports -> $OUT', file=sys.stderr)
"

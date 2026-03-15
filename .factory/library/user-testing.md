# User Testing

## Validation Surface

**Testing Surface:** Rust unit tests via cargo test
- No UI or browser testing needed (pure library)
- Validation through test assertions and BopDs queries

**Tools:**
- cargo test -p truck-bop
- cargo clippy -p truck-bop
- proptest for property-based tests

**Setup:** None required

## Validation Concurrency

**Surface:** cargo test
**Per-validator cost:** ~100-200 MB RAM
**Max concurrent validators:** 5
**Rationale:** Conservative estimate with 70% headroom on 16 GB machine (10 CPU cores)

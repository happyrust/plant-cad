# User Testing

Testing surface and validation approach for truck-bop.

**What belongs here:** Test execution details, validation surface info, resource constraints.

---

## Validation Surface

**Surface:** Command-line (cargo test)

**Tools:**
- cargo test -p truck-bop
- cargo check -p truck-bop
- cargo clippy -p truck-bop

**Approach:**
- Unit tests verify individual algorithms
- Integration tests verify end-to-end boolean operations
- TDD: write failing tests first, then implement

## Validation Concurrency

**Max concurrent validators:** 5

**Rationale:** Tests are CPU-bound. Machine has 12 cores with sufficient memory. 5 concurrent test processes provide good parallelism without overloading.

**Resource profile:**
- Each test process: ~200MB RAM, 1-2 CPU cores
- Total for 5 concurrent: ~1GB RAM, 5-10 cores
- Well within machine capacity

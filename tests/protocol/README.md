# Protocol tests

Run from the repository root:

```sh
python3 tests/protocol/run_tests.py
```

The runner compiles the real protocol sources with a single-threaded Linux/serial shim and AddressSanitizer/UndefinedBehaviorSanitizer. It generates temporary table configurations that delete the first, middle, last, several, or all feature entries; it also removes configuration/work-parameter entries and reverses table order.

The harness exercises request encoding, fragmented byte reception, CRC, response matching, exception responses, permission checks, sparse feature reads, unchanged physical addresses, field masks, zeroing removed fields, failed-read cache preservation, and request ownership until the waiter consumes its result.

This does not test actual hardware, workqueue scheduling, or concurrent kernel execution. The deliberately unresolved unsolicited-frame handler has a test-only stub; production code keeps the requested TODO and compile error.

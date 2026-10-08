# Acquisition tests

Run `python3 tests/acquisition/run_tests.py` from the repository root.

The harness compiles the production acquisition state machine and policy with
AddressSanitizer/UndefinedBehaviorSanitizer, a deterministic clock, and protocol,
IIO and thread substitutes. It exercises startup delay, baud matching/switching,
verification before publishing, adaptive windows and bounds, read-time headroom,
padding-independent comparisons, timestamp wraparound, communication-failure
thresholds, stale cache invalidation, sensor resets, exponential retry caps,
Modbus exceptions, manual baud changes, lost write acknowledgments, buffer errors,
low-baud response budgets, thread creation failure and shutdown guards.

These are deterministic state-machine tests. They do not model concurrent kernel
scheduling, UART/firmware timing, or real IIO buffer consumers. Build the module
against the target kernel and validate the device's 40102 change timing on hardware.

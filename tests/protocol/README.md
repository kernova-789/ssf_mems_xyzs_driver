# Protocol tests

Run from the repository root:

```sh
python3 tests/protocol/run_tests.py
```

The runner compiles each real protocol source as an independent translation unit and links them with a single-threaded Linux/serial shim and AddressSanitizer/UndefinedBehaviorSanitizer. Each public header is also compiled on its own. Object-symbol checks enforce the dependency direction: the codec cannot call upper layers, receive cannot call request/business directly, and request cannot call receive/business. The codec's generated dependency file is checked for upper-layer state headers.

Temporary table configurations delete the first, middle, last, several, or all feature entries; they also remove configuration/work-parameter entries and reverse table order. Production files are never changed by the runner.

The harness exercises independent module initialization, FIFO allocation failure, partial-frame cleanup, rejection of new work after receive shutdown, complete-frame callback/context delivery, request encoding, fragmented byte reception through the receive work item, CRC, response matching, exception responses, permission checks, sparse feature reads, unchanged physical addresses, field masks, zeroing removed fields, failed-read cache preservation, and request ownership until the waiter consumes its result. Unclaimed frames must be discarded without changing request state, output buffers, feature caches, or wake counts; completed frame buffers and slots must be released, and a subsequent matching response must still complete the request.

Baud-rate cases cover a direct 40102 read, switching through unique enum rates,
skipping the duplicate default/9600 value, keeping the discovered rate,
restoring the entry rate after a full timeout, invalid register values, write
echo errors, and synchronizing the simulated sensor and serdev after a write.

This does not test actual hardware, workqueue scheduling, or concurrent kernel execution. The tests link the production frame-dispatch implementation directly; no fallback-handler stub is used.

Startup serial configuration tests cover default properties, none/odd/even
mapping (the Linux enum order differs from the sensor enum), slave-ID bounds,
unsupported baud rates, logged defaults for missing/unreadable properties,
independent per-property fallback, invalid parity enums, controller
baud/parity failures, and recording a rounded actual host rate. Startup parsing
and configuration must not send any sensor commands. The existing baud-write
simulation still models immediate application; it is a regression fixture, not
validation of the firmware's save/reset workflow.

# Protocol tests

Run from the repository root:

```sh
python3 tests/protocol/run_tests.py
```

The runner compiles each real protocol source as an independent translation unit and links them with a single-threaded Linux/serial shim and AddressSanitizer/UndefinedBehaviorSanitizer. Each public header is also compiled on its own. Object-symbol checks enforce the dependency direction: the codec cannot call upper layers, receive cannot call request/business directly, and request cannot call receive/business. The codec's generated dependency file is checked for upper-layer state headers.

Temporary table configurations delete the first, middle, last, several, or all feature entries; they also remove configuration/work-parameter entries and reverse table order. Production files are never changed by the runner.

The harness exercises independent module initialization, FIFO allocation failure, partial-frame cleanup, rejection of new work after receive shutdown, complete-frame callback/context delivery, request encoding, fragmented byte reception through the receive work item, CRC, response matching, exception responses, permission checks, sparse feature reads, unchanged physical addresses, field masks, zeroing removed fields, failed-read cache preservation, and request ownership until the waiter consumes its result. Unclaimed frames must be discarded without changing request state, output buffers, feature caches, or wake counts; completed frame buffers and slots must be released, and a subsequent matching response must still complete the request.

This does not test actual hardware, workqueue scheduling, or concurrent kernel execution. The tests link the production frame-dispatch implementation directly; no fallback-handler stub is used.

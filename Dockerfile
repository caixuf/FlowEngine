# FlowEngine Docker Image
# Build:  docker build -t flowengine .
# Run:    docker run --rm flowengine
# Demo:   docker run --rm flowengine demo 15
# Shell:  docker run --rm -it flowengine bash

FROM ubuntu:24.04 AS builder

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake g++ \
    libcjson-dev python3 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /flowengine
COPY . .

RUN cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
    && cmake --build build -j$(nproc)

# ── Runtime image ─────────────────────────────────────────
FROM ubuntu:24.04

RUN apt-get update && apt-get install -y --no-install-recommends \
    libcjson-dev python3 curl \
    && rm -rf /var/lib/apt/lists/*

COPY --from=builder /flowengine/build/bin /usr/local/bin
COPY --from=builder /flowengine/build/lib /usr/local/lib/flowengine/plugins
COPY --from=builder /flowengine/tools /usr/local/share/flowengine/tools
COPY --from=builder /flowengine/include /usr/local/include/flowengine
COPY --from=builder /flowengine/scripts /usr/local/share/flowengine/scripts

ENV PATH="/usr/local/share/flowengine/tools:/usr/local/share/flowengine/scripts:$PATH"

# Default: run demo
ENTRYPOINT ["flow_e2e"]
CMD ["15"]

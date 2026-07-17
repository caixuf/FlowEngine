#!/usr/bin/env python3
"""
flowcoro_lockfree_fix.py — 修复 flowcoro 的 lockfree::Queue 多消费者竞态缺陷

问题：
  flowcoro 的 `lockfree::Queue<T>` 号称无锁 MPMC 队列，但 dequeue_unsafe
  在多消费者并发场景下存在 ABA / use-after-free：
    1. 消费者 A 与 B 同时读到同一个 head_node 与 next；
    2. 二者都 `next->data.exchange(nullptr)`，败者返回 false；
    3. 胜者 CAS 推进 head 并立即 pool_free(head_node)；
    4. 生产者 C 紧接着 pool_malloc 复用同一块内存，执行
       `new (&block->node) Node()` 写入 next 字段；
    5. 败者（或另一消费者）此刻仍在读取 head_node->next —— 同一内存！
    6. TSAN 报 data race；release 下表现为间歇性堆损坏（free/malloc 崩溃）。

修复策略：
  将 Queue 替换为 std::mutex + std::condition_variable + std::queue 的
  阻塞互斥实现。线程池的任务投递频率远低于任务执行本身，互斥开销可忽略，
  换来绝对的正确性。保留原公开接口（enqueue / dequeue / empty /
  size_estimate）与 destroyed 标志语义，不改动 Stack/RingBuffer/AtomicCounter。

应用方式：CMake FetchContent PATCH_COMMAND 调用本脚本（cwd 为 flowcoro 源根）。
幂等：多次执行结果一致；若已修补则无操作。
"""

import os
import re
import sys

TARGET = os.path.join("include", "flowcoro", "lockfree.h")

# 正确实现：互斥保护的 FIFO 队列。接口与原 lockfree::Queue 完全一致。
FIXED_QUEUE = r'''// 互斥保护的 FIFO 队列（修正原"无锁"实现的 MPMC ABA / UAF 竞态）
// 保留 enqueue/dequeue/empty/size_estimate 接口与 destroyed 标志语义。
template<typename T>
class Queue {
private:
    mutable std::mutex      mtx_;
    std::queue<T>           q_;
    alignas(64) std::atomic<bool> destroyed{false};

public:
    Queue() {
        destroyed.store(false, std::memory_order_release);
    }

    ~Queue() {
        destroyed.store(true, std::memory_order_release);
        std::lock_guard<std::mutex> lk(mtx_);
        while (!q_.empty()) q_.pop();
    }

    void enqueue(T item) {
        if (destroyed.load(std::memory_order_acquire)) {
            return; // 队列已析构，丢弃任务
        }
        {
            std::lock_guard<std::mutex> lk(mtx_);
            if (destroyed.load(std::memory_order_acquire)) return;
            q_.push(std::move(item));
        }
    }

    bool dequeue(T& result) {
        if (destroyed.load(std::memory_order_acquire)) {
            return false; // 队列已析构
        }
        std::lock_guard<std::mutex> lk(mtx_);
        if (q_.empty()) return false;
        result = std::move(q_.front());
        q_.pop();
        return true;
    }

    bool empty() const {
        std::lock_guard<std::mutex> lk(mtx_);
        return q_.empty();
    }

    size_t size_estimate() const {
        std::lock_guard<std::mutex> lk(mtx_);
        return q_.size();
    }
};
'''


def find_class_block(text, class_name):
    """返回 (start, end) 字节偏移，覆盖 `template<...>\\nclass <class_name> {` 起
    到与之匹配的顶层 `};`（含）。找不到返回 None。"""
    m = re.search(r'template\s*<[^>]*>\s*\nclass\s+' + re.escape(class_name) + r'\s*\{',
                  text)
    if not m:
        return None
    start = m.start()
    depth = 0
    i = m.end() - 1  # 指向 '{'
    while i < len(text):
        c = text[i]
        if c == '{':
            depth += 1
        elif c == '}':
            depth -= 1
            if depth == 0:
                # 包含紧随其后的 ';'
                j = i + 1
                while j < len(text) and text[j] in ' \t':
                    j += 1
                if j < len(text) and text[j] == ';':
                    return (start, j + 1)
                return (start, i + 1)
        i += 1
    return None


def main():
    path = TARGET
    if not os.path.isfile(path):
        print(f"[flowcoro-fix] target not found: {path}", file=sys.stderr)
        return 1
    with open(path, 'r', encoding='utf-8', errors='replace') as f:
        text = f.read()

    if '互斥保护的 FIFO 队列（修正原' in text:
        print(f"[flowcoro-fix] already patched: {path}")
        return 0

    block = find_class_block(text, "Queue")
    if block is None:
        print(f"[flowcoro-fix] Queue class not found in {path}", file=sys.stderr)
        return 1

    start, end = block
    new_text = text[:start] + FIXED_QUEUE + text[end:]
    with open(path, 'w', encoding='utf-8') as f:
        f.write(new_text)
    print(f"[flowcoro-fix] patched Queue in {path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

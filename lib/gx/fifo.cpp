#include "fifo.hpp"
#include "command_processor.hpp"
#include "../internal.hpp"

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <cstring>

namespace aurora::gx::fifo {
static Module Log("aurora::gx::fifo");

namespace detail {
uint8_t* sBufferData = nullptr;
uint32_t sBufferSize = 0;
uint32_t sBufferCapacity = 0;
bool sInDisplayList = false;
uint8_t* sDlBuffer = nullptr;
uint32_t sDlSize = 0;
uint32_t sDlWritePos = 0;
} // namespace detail

void init() {
  constexpr uint32_t initialCapacity = 64 * 1024;
  free(detail::sBufferData);
  detail::sBufferData = static_cast<uint8_t*>(malloc(initialCapacity));
  detail::sBufferSize = 0;
  detail::sBufferCapacity = initialCapacity;
  detail::sInDisplayList = false;
  detail::sDlBuffer = nullptr;
  detail::sDlSize = 0;
  detail::sDlWritePos = 0;
}

/**
 * Grow the FIFO to fit `length` more bytes.
 *
 * ★ This used to be four lines of uint32 arithmetic, and it turned a bug elsewhere into an
 * unkillable hang. Read before simplifying it back.
 *
 * A non-terminating draw loop (a cycle in a J3DDrawBuffer packet list will do it) writes GX
 * commands forever, so this doubled 64 KiB -> ... -> 2 GiB. At that point `sBufferCapacity * 2`
 * OVERFLOWED uint32 - 2,151,079,386 * 2 wraps to 7,191,476 - so `newCap` collapsed to exactly
 * `needed`, and amortised doubling degenerated into "realloc and memcpy 2 GiB on every single
 * write". The process then span in the allocator forever: never crashing, never running out of
 * memory, just pegging a core. It was diagnosed only by reading these two globals out of a live
 * hung process, which is a bad way to find out.
 *
 * So: compute in 64-bit, bound the result, and check the allocation.
 *
 * The bound is the emulated console's own MEM1 size rather than a number invented here. A frame's
 * command stream cannot legitimately exceed the entire RAM of the machine being emulated - real
 * hardware's FIFO is a few hundred KiB - so anything past that is a runaway by definition, and
 * tying it to `g_config.mem1Size` keeps one place saying how big the console is. A host that never
 * configured one falls back to the only other real ceiling there is: what these uint32 counters
 * can address.
 */
void write_data_grow(const void* data, uint32_t length) {
  const uint64_t needed = static_cast<uint64_t>(detail::sBufferSize) + length;
  const uint64_t limit =
      g_config.mem1Size != 0 ? static_cast<uint64_t>(g_config.mem1Size) : UINT32_MAX;

  if (needed > limit) {
    // Fatal on purpose. Returning would silently drop commands and draw a corrupt frame, and
    // carrying on is what produced the original hang. Say which frame died and stop.
    FATAL("gx::fifo: command stream reached {} bytes, past the {} byte limit - a draw loop is not "
          "terminating (a cycle in a draw-buffer packet list will do this). Refusing to grow.",
          needed, limit);
  }

  const uint64_t doubled = static_cast<uint64_t>(detail::sBufferCapacity) * 2;
  const uint64_t newCap = std::min(std::max(doubled, needed), limit);

  auto* grown = static_cast<uint8_t*>(realloc(detail::sBufferData, newCap));
  if (grown == nullptr) {
    // realloc leaves the original block valid, so the old pointer is still live - but there is no
    // way forward, and the previous code dereferenced the null straight away.
    FATAL("gx::fifo: out of memory growing the command stream to {} bytes", newCap);
  }

  detail::sBufferData = grown;
  std::memcpy(detail::sBufferData + detail::sBufferSize, data, length);
  detail::sBufferSize = static_cast<uint32_t>(needed);
  detail::sBufferCapacity = static_cast<uint32_t>(newCap);
}

void patch_u32(const uint32_t offset, const uint32_t val) {
  ASSERT(!detail::sInDisplayList && offset + sizeof(uint32_t) <= detail::sBufferSize,
         "fifo::patch_u32: invalid patch offset {} (buffer size {})", offset, detail::sBufferSize);
  const auto out = bswap(val);
  std::memcpy(detail::sBufferData + offset, &out, sizeof(out));
}

void begin_display_list(uint8_t* buf, uint32_t size) {
  detail::sInDisplayList = true;
  detail::sDlBuffer = buf;
  detail::sDlSize = size;
  detail::sDlWritePos = 0;
}

uint32_t end_display_list() {
  detail::sInDisplayList = false;
  uint32_t bytesWritten = detail::sDlWritePos;
  uint32_t padded = (bytesWritten + 31) & ~31u;
  while (detail::sDlWritePos < padded && detail::sDlWritePos < detail::sDlSize) {
    detail::sDlBuffer[detail::sDlWritePos++] = 0;
  }
  detail::sDlBuffer = nullptr;
  detail::sDlSize = 0;
  detail::sDlWritePos = 0;
  return padded;
}

bool in_display_list() { return detail::sInDisplayList; }

void drain() {
  if (detail::sBufferSize == 0) {
    return;
  }
  process(detail::sBufferData, detail::sBufferSize, true);
  detail::sBufferSize = 0;
}

const uint8_t* get_buffer_data() { return detail::sBufferData; }
uint32_t get_buffer_size() { return detail::sBufferSize; }
void clear_buffer() { detail::sBufferSize = 0; }

} // namespace aurora::gx::fifo

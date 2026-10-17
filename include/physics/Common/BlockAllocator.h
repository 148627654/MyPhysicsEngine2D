#pragma once
#include <cstddef>
#include <cstdint>


class BlockAllocator {
public:
    static const int CHUNK_SIZE = 16 * 1024;      // 16KB 大页
    static const int MAX_BLOCK_SIZE = 1024;       // 支持的最大池化规格
    static const int BLOCK_SIZES_COUNT = 14;      // 14 级尺寸阶梯

    BlockAllocator();
    ~BlockAllocator();

    // 核心分配与回收接口
    void* Allocate(int size);
    void Free(void* p, int size);

    // 安全清空：释放全部大页 + 兜底回收所有未归还的超大对象（零泄漏）
    void Clear();

private:
    struct Block {
        Block* next;
    };

    struct Chunk {
        int blockSize;
        Block* blocks;
    };

    // 超大对象回退登记表（Allocate 时登记，Free/Clear 时摘除）
    struct FallbackBlock {
        void* ptr;
    };

    int m_chunkSpace;
    int m_chunkCount;
    Chunk* m_chunks;

    int m_fallbackSpace;
    int m_fallbackCount;
    FallbackBlock* m_fallbacks;

    Block* m_freeLists[BLOCK_SIZES_COUNT];

    static int s_blockSizes[BLOCK_SIZES_COUNT];
    static uint8_t s_blockSizeLookup[MAX_BLOCK_SIZE + 1];
    static bool s_lookupInitialized;
};

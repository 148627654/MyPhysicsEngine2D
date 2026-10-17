#include "BlockAllocator.h"
#include <cstdlib>

int BlockAllocator::s_blockSizes[BLOCK_SIZES_COUNT] = {
	16, 32, 48, 64, 80, 96, 112, 128, 160, 224, 320, 448, 640, 1024
};

uint8_t BlockAllocator::s_blockSizeLookup[MAX_BLOCK_SIZE + 1];
bool BlockAllocator::s_lookupInitialized = false;

BlockAllocator::BlockAllocator()
{
    // 检查静态尺寸映射表是否已初始化，若未初始化则构建查表 (单例初始化)
    if (!s_lookupInitialized) {
        int j = 0;
        s_blockSizeLookup[0] = 0;
        for (int i = 1; i <= MAX_BLOCK_SIZE; ++i) {
            if (i > s_blockSizes[j]) {
                ++j;
            }
            s_blockSizeLookup[i] = static_cast<uint8_t>(j);
        }
        s_lookupInitialized = true;
    }

    // 初始化大页记录器 (初始预留 32 个 Chunk 槽位)
    m_chunkCount = 0;
    m_chunkSpace = 32;
    m_chunks = (Chunk*)std::malloc(m_chunkSpace * sizeof(Chunk));

    // 初始化超大对象回退登记表 (初始预留 16 个槽位)
    m_fallbackCount = 0;
    m_fallbackSpace = 16;
    m_fallbacks = (FallbackBlock*)std::malloc(m_fallbackSpace * sizeof(FallbackBlock));

    // 将 14 个规格桶的空闲链表头指针全部初始化为 nullptr (开局全空)
    for (int i = 0; i < BLOCK_SIZES_COUNT; ++i) {
        m_freeLists[i] = nullptr;
    }
}

BlockAllocator::~BlockAllocator()
{
	Clear();
}

void* BlockAllocator::Allocate(int size)
{
    if (size <= 0)
        return nullptr;

    // 超大对象直接走系统堆分配，并登记在案以便 Clear() 统一兜底回收
    if (size > MAX_BLOCK_SIZE)
    {
        void* p = std::malloc(size);
        if (p == nullptr)
            return nullptr;

        if (m_fallbackCount >= m_fallbackSpace)
        {
            m_fallbackSpace *= 2;
            m_fallbacks = (FallbackBlock*)std::realloc(m_fallbacks, m_fallbackSpace * sizeof(FallbackBlock));
        }
        m_fallbacks[m_fallbackCount].ptr = p;
        m_fallbackCount++;
        return p;
    }

    // 极致 O(1) 查表：瞬间定位规格桶索引
    int index = s_blockSizeLookup[size];

    // 快路径 (Fast Path)：池子里有现成空闲块，直接弹出！
    Block* block = m_freeLists[index];
    if (block != nullptr)
    {
        m_freeLists[index] = block->next;
        return block;
    }

    // 慢路径 (Slow Path)：当前规格桶空了，从操作系统批发一个 16KB Chunk 切片。
    // Clear() 之后 m_chunks 已释放，这里懒重建记录数组，保证分配器可复用
    if (m_chunks == nullptr)
    {
        m_chunkSpace = 32;
        m_chunks = (Chunk*)std::malloc(m_chunkSpace * sizeof(Chunk));
    }
    if (m_chunkCount >= m_chunkSpace)
    {
        m_chunkSpace *= 2;
        m_chunks = (Chunk*)std::realloc(m_chunks, m_chunkSpace * sizeof(Chunk));
    }

    Chunk* chunk = m_chunks + m_chunkCount;
    m_chunkCount++;

    chunk->blockSize = s_blockSizes[index];
    chunk->blocks = (Block*)std::malloc(CHUNK_SIZE); // 申请 16KB

    // 切片与就地穿线
    int blockSize = chunk->blockSize;
    int blockCount = CHUNK_SIZE / blockSize;

    int8_t* p = (int8_t*)chunk->blocks;
    for (int i = 0; i < blockCount - 1; ++i) {
        Block* current = (Block*)(p + i * blockSize);
        Block* next = (Block*)(p + (i + 1) * blockSize);
        current->next = next;
    }
    Block* last = (Block*)(p + (blockCount - 1) * blockSize);
    last->next = nullptr;

    // 弹出首块返回，剩余链表挂回桶中备用
    m_freeLists[index] = ((Block*)p)->next;
    return p;
}

void BlockAllocator::Free(void* p, int size)
{
    // 空指针防御
    if (p == nullptr || size <= 0)
    {
        return;
    }

    // 当初走系统 malloc 分配的：从回退登记表摘除（交换删除），再还给系统
    if (size > MAX_BLOCK_SIZE)
    {
        for (int i = 0; i < m_fallbackCount; ++i)
        {
            if (m_fallbacks[i].ptr == p)
            {
                m_fallbacks[i] = m_fallbacks[m_fallbackCount - 1];
                m_fallbackCount--;
                break;
            }
        }
        std::free(p);
        return;
    }

    // O(1) 查表定位对应的规格桶
    int index = s_blockSizeLookup[size];

    // 极致 O(1) 归还：头插法塞回空闲链表
    Block* block = static_cast<Block*>(p);
    block->next = m_freeLists[index];
    m_freeLists[index] = block;
}

void BlockAllocator::Clear()
{
    // 逐一释放所有向操作系统申请的 16KB 物理大页 (Chunk)
    for (int i = 0; i < m_chunkCount; ++i)
    {
        std::free(m_chunks[i].blocks);
    }

    // 释放存放 Chunk 记录的动态数组本身
    std::free(m_chunks);
    m_chunks = nullptr;
    m_chunkCount = 0;
    m_chunkSpace = 0;

    // 兜底回收所有尚未归还的超大对象（Allocate 后未 Free 也绝不泄漏）
    for (int i = 0; i < m_fallbackCount; ++i)
    {
        std::free(m_fallbacks[i].ptr);
    }

    // 释放回退登记表数组本身
    std::free(m_fallbacks);
    m_fallbacks = nullptr;
    m_fallbackCount = 0;
    m_fallbackSpace = 0;

    // 重置所有 14 个规格桶的链表头指针
    for (int i = 0; i < BLOCK_SIZES_COUNT; ++i)
    {
        m_freeLists[i] = nullptr;
    }
}

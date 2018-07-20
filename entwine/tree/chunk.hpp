/******************************************************************************
* Copyright (c) 2016, Connor Manning (connor@hobu.co)
*
* Entwine -- Point cloud indexing
*
* Entwine is available under the terms of the LGPL2 license. See COPYING
* for specific license text and more information.
*
******************************************************************************/

#pragma once

#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <vector>

#include <pdal/PointTable.hpp>

#include <entwine/formats/cesium/tile-info.hpp>
#include <entwine/formats/cesium/util.hpp>
#include <entwine/tree/climber.hpp>
#include <entwine/types/dim-info.hpp>
#include <entwine/types/point.hpp>
#include <entwine/types/schema.hpp>
#include <entwine/types/tube.hpp>

namespace entwine
{

namespace arbiter { class Endpoint; }

class Builder;
class Metadata;
class Unpacker;

class Chunk
{
    // friend class ChunkStorage;
    friend class Builder;
    friend class Storage;

public:
    Chunk(
            const Builder& builder,
            const Bounds& bounds,
            std::size_t depth,
            const Id& id,
            const Id& maxPoints);

    Chunk(Chunk&& other) = default;

    virtual ~Chunk();

    static std::unique_ptr<Chunk> create(
            const Builder& builder,
            const Bounds& bounds,
            std::size_t depth,
            const Id& id,
            const Id& maxPoints,
            bool exists);

    const Id& maxPoints() const { return m_maxPoints; }
    const Id& id() const { return m_id; }
    const Builder& builder() const { return m_builder; }
    const Metadata& metadata() const { return m_metadata; }
    const Storage& storage() const { return m_metadata.storage(); }
    const Bounds& bounds() const { return m_bounds; }

    virtual void save();

    virtual ChunkType type() const = 0;
    const Schema& schema() const { return m_metadata.schema(); }
    PointPool& pool() { return m_pointPool; }

    virtual Cell::PooledStack acquire() = 0;

    Tube::Insertion insert(const Climber& climber, Cell::PooledNode& cell)
    {
        return getTube(climber).insert(climber, cell);
    }

    static std::size_t count();

    virtual cesium::TileInfo info() const = 0;

protected:
    virtual void populate(Cell::PooledStack cells);

    virtual Tube& getTube(const Climber& climber) = 0;

    std::size_t divisor() const
    {
        const auto& s(m_metadata.structure());
        std::size_t d(1 << s.nominalChunkDepth());

        if (m_depth > s.sparseDepthBegin())
        {
            d <<= m_depth - s.sparseDepthBegin();
        }

        return d;
    }

    Id endId() const { return m_id + m_maxPoints; }

    virtual void tile() const { }

    const Builder& m_builder;
    const Metadata& m_metadata;
    const Bounds m_bounds;
    PointPool& m_pointPool;

    const std::size_t m_depth;
    const std::size_t m_zDepth;
    Id m_id;

    Id m_maxPoints;
};

class SparseChunk : public Chunk
{
public:
    SparseChunk(
            const Builder& builder,
            const Bounds& bounds,
            std::size_t depth,
            const Id& id,
            const Id& maxPoints,
            bool exists);

    virtual ~SparseChunk();

    virtual ChunkType type() const override { return ChunkType::Sparse; }
    virtual cesium::TileInfo info() const override;

private:
    virtual Cell::PooledStack acquire() override;

    virtual void tile() const override;

    virtual Tube& getTube(const Climber& climber) override
    {
        const Id norm(normalize(climber.index()));

        std::lock_guard<std::mutex> lock(m_mutex);
        return m_tubes[norm];
    }

    Id normalize(const Id& rawIndex) const
    {
        assert(rawIndex >= m_id);
        assert(rawIndex < endId());

        return rawIndex - m_id;
    }

    std::map<Id, Tube> m_tubes;
    std::mutex m_mutex;
};

class ContiguousChunk : public Chunk
{
    friend class BaseChunk;

public:
    ContiguousChunk(
            const Builder& builder,
            const Bounds& bounds,
            std::size_t depth,
            const Id& id,
            const Id& maxPoints,
            bool exists);

    ContiguousChunk(ContiguousChunk&& other) = default;

    virtual ChunkType type() const override { return ChunkType::Contiguous; }

    virtual cesium::TileInfo info() const override;

    bool empty() const
    {
        for (const auto& tube : m_tubes)
        {
            if (!tube.empty()) return false;
        }
        return true;
    }

protected:
    virtual Cell::PooledStack acquire() override;

    virtual void tile() const override;

    virtual Tube& getTube(const Climber& climber) override
    {
        return m_tubes.at(normalize(climber.index()));
    }

    std::size_t normalize(const Id& rawIndex) const
    {
        assert(rawIndex >= m_id);
        assert(rawIndex < endId());

        return (rawIndex - m_id).getSimple();
    }

    void append(ContiguousChunk& other)
    {
        m_tubes.insert(
                m_tubes.end(),
                std::make_move_iterator(other.m_tubes.begin()),
                std::make_move_iterator(other.m_tubes.end()));

        m_maxPoints += other.maxPoints();
    }

    void clear()
    {
        m_id = endId();
        m_tubes.clear();
        m_maxPoints = 0;
    }

    std::vector<Tube> m_tubes;
};

class BaseChunk : public Chunk
{
public:
    BaseChunk(const Builder& builder, bool exists);

    std::set<Id> merge(BaseChunk& other);

    virtual cesium::TileInfo info() const override;
    virtual ChunkType type() const override { return ChunkType::Contiguous; }
    std::vector<cesium::TileInfo> baseInfo() const;

    virtual void save() override;

private:
    virtual Cell::PooledStack acquire() override
    {
        std::cout << "No BaseChunk::acquire" << std::endl;
        throw std::runtime_error("No BaseChunk::acquire");
    }
    virtual void populate(Cell::PooledStack cells) override
    {
        std::cout << "No BaseChunk::populate" << std::endl;
        throw std::runtime_error("No BaseChunk::populate");
    }

    virtual void tile() const override;

    virtual Tube& getTube(const Climber& climber) override
    {
        return m_chunks.at(climber.depth()).getTube(climber);
    }

    std::vector<ContiguousChunk> m_chunks;
};

} // namespace entwine


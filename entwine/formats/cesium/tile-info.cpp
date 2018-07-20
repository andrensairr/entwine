/******************************************************************************
* Copyright (c) 2016, Connor Manning (connor@hobu.co)
*
* Entwine -- Point cloud indexing
*
* Entwine is available under the terms of the LGPL2 license. See COPYING
* for specific license text and more information.
*
******************************************************************************/

#include <entwine/formats/cesium/tile-info.hpp>

#include <entwine/formats/cesium/settings.hpp>
#include <entwine/formats/cesium/util.hpp>
#include <entwine/third/arbiter/arbiter.hpp>
#include <entwine/types/metadata.hpp>
#include <entwine/types/structure.hpp>
#include <entwine/util/matrix.hpp>
#include <entwine/util/io.hpp>

namespace entwine
{
namespace cesium
{
namespace
{
	std::size_t foundTileCount(8);
}

void TileInfo::write(
        const Metadata& metadata,
        const arbiter::Endpoint& endpoint,
        double geometricError) const
{
    Json::Value json;

    json["asset"]["version"] = "0.0";
    json["geometricError"] = geometricError;

    std::cout << "Aggregating root" << std::endl;
    insertInto(
            json["root"],
            metadata,
            endpoint,
            geometricError / 2.0,
            metadata.structure().baseDepthBegin());

    json["root"]["refine"] = "add";

    if (const auto* t = metadata.transformation())
    {
        const auto columnMajorInverse(matrix::flip(matrix::inverse(*t)));
        for (const auto d : columnMajorInverse)
        {
            json["root"]["transform"].append(d);
        }
    }

    std::cout << "Writing root" << std::endl;
	std::cout << "Wrote " << foundTileCount << " of " << size() << " tiles to tileset.json." << std::endl;
    io::ensurePut(endpoint, "tileset.json", json.toStyledString());
}

bool TileInfo::restart(
        const Metadata& metadata,
        const arbiter::Endpoint& endpoint,
        const double geometricError,
        const std::size_t depth,
        const std::size_t tick) const
{
    Json::Value json;

    json["asset"]["version"] = "0.0";
    json["geometricError"] = geometricError;

    if (insertInto(
                json["root"],
                metadata,
                endpoint,
                geometricError,
                depth,
                tick))
    {
        io::ensurePut(
                endpoint,
                "tileset-" + m_id.str() + "-" + std::to_string(tick) + ".json",
                json.toStyledString());

        return true;
    }

    return false;
}

Bounds TileInfo::conformingBounds(
        const Metadata& metadata,
        const std::size_t tick) const
{
	const auto& s(metadata.structure());
	const double dblTick(tick);
    std::size_t intTick(1 << (m_depth - s.nominalChunkDepth()));

    if (m_depth > s.sparseDepthBegin())
    {
        intTick >>= m_depth - s.sparseDepthBegin();
    }

    const double maxTick(intTick);

    return Bounds(
            m_bounds.min().x,
            m_bounds.min().y,
            m_bounds.min().z + m_bounds.height() / maxTick * dblTick,
            m_bounds.max().x,
            m_bounds.max().y,
            m_bounds.min().z + m_bounds.height() / maxTick * (dblTick + 1.0));
}

bool TileInfo::insertInto(
        Json::Value& json,
        const Metadata& m,
        const arbiter::Endpoint& endpoint,
        const double geometricError,
        const std::size_t depth,
        const std::size_t tick) const
{
    bool found(false);

    const std::size_t nextDepth(depth + 1);
    const double nextGeometricError(geometricError / 2.0);

	if (m_ticks.count(tick))
	{
		if (depth >= m.structure().coldDepthBegin() || !tick)
		{
			found = true;

			json["boundingVolume"] = cesium::boundingVolumeJson(
				depth >= m.structure().coldDepthBegin() ?
				conformingBounds(m, tick) : m.boundsNativeCubic());

			json["geometricError"] = geometricError;

			json["content"]["url"] =
				m_id.str() + "-" + std::to_string(tick) + ".pnts";
		}

		for (const auto& c : m_children)
		{
			const auto& child(*c.second);
			auto nextTick(tick);
			auto insertChildOrRestart([&]()
			{
				Json::Value next;

				const auto fromBase(nextDepth - m.structure().baseDepthBegin());

				if (fromBase % m.cesiumSettings()->tilesetSplit() == 0)
				{
					// Insert links out to the child metadata files.
					next["boundingVolume"] =
						cesium::boundingVolumeJson(
							child.conformingBounds(m, nextTick));

					next["content"]["url"] =
						"tileset-" + child.id().str() + "-" +
						std::to_string(nextTick) + ".json";

					next["geometricError"] = nextGeometricError;

					// Gather and write the contents of those files.
					if (child.restart(
						m,
						endpoint,
						nextGeometricError,
						nextDepth,
						nextTick))
					{
						json["children"].append(next);
					}
				}
				else if (
					child.insertInto(
						next,
						m,
						endpoint,
						nextGeometricError,
						nextDepth,
						nextTick))
				{
					json["children"].append(next);
				}
			});

			if (child.m_ticks.count(tick))
			{
				insertChildOrRestart();
			}
			else
			{
				const std::size_t baseTick(tick * 2);

				for (nextTick = baseTick; nextTick < baseTick + 2; ++nextTick)
				{
					//for (const auto& t : m_ticks)
					//{
					//	const std::size_t nextTick(t.first);
					insertChildOrRestart();
				}
			}
		}
	}

	if (found)
		foundTileCount++;

    return found;
}


} // namespace cesium
} // namespace entwine

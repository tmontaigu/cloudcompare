#include "PDALConverters.h"

#include <ccScalarField.h>

#include <utility>


static bool isCoordsComponent(const pdal::Dimension::Id id)
{
	using namespace pdal::Dimension;
	return (id == Id::X) || (id == Id::Y) || (id == Id::Z);
}
static bool isColorComponent(const pdal::Dimension::Id id)
{
	using namespace pdal::Dimension;
	return (id == Id::Red) || (id == Id::Green) || (id == Id::Blue);
}

static bool isScalarField(const pdal::Dimension::Id id)
{
	return !isCoordsComponent(id) && !isColorComponent(id);
}

static ScalarFieldMap createScalarFieldMap(const pdal::Dimension::IdList& dims, const size_t pointCount)
{
	ScalarFieldMap scalarFieldMap;
	for (const pdal::Dimension::Id id : dims)
	{
		if (!isScalarField(id))
		{
			continue;
		}
		const std::string name = pdal::Dimension::name(id);
		auto currentScalarField = new ccScalarField(name.c_str());
		if (currentScalarField->reserveSafe(pointCount))
		{
			scalarFieldMap[id] = currentScalarField;
		}
		else
		{
			ccLog::Warning(QString("[LAS] Not enough memory: '%1' field will be ignored!").arg(name.c_str()));
			currentScalarField->release();
			break;
		}
	}
	return scalarFieldMap;
}

CCVector3 convertPointFromPDAL(const pdal::PointRef& point, const CCVector3d& shift = CCVector3d(0.,0.,0.))
{
	return CCVector3{
		static_cast<PointCoordinateType>(point.getFieldAs<double>(pdal::Dimension::Id::X) + shift.x),
		static_cast<PointCoordinateType>(point.getFieldAs<double>(pdal::Dimension::Id::Y) + shift.y),
		static_cast<PointCoordinateType>(point.getFieldAs<double>(pdal::Dimension::Id::Z) + shift.z)
	};
}

PDALToCC::PDALToCC(pdal::QuickInfo  info, ProgressCallBack  cb)
	: m_info(std::move(info))
	, m_progressCallback(std::move(cb))
{
}

bool PDALToCC::processOne(pdal::PointRef& p)
{
	if (m_isFirstPoint)
	{
		CCVector3d ccPoint(
			p.getFieldAs<double>(pdal::Dimension::Id::X),
			p.getFieldAs<double>(pdal::Dimension::Id::Y),
			p.getFieldAs<double>(pdal::Dimension::Id::Z)
		);
		m_determineShiftCallback(ccPoint, m_cloud.get());
		m_isFirstPoint = false;
	}
	const CCVector3d& shift = m_cloud->getGlobalShift();
	m_cloud->addPoint(convertPointFromPDAL(p, shift));

	for (ScalarFieldMap::value_type& mapItem : m_scalarFieldMap)
	{
		auto value = p.getFieldAs<ScalarType>(mapItem.first);
		mapItem.second->addElement(value);
	}

	m_progressCallback();
	return false;
}

void PDALToCC::prepared(pdal::PointTableRef table)
{
	ccLog::Print("[PDALFilter] Initialize");
	// FIXME hum
	if (!m_cloud->reserve(m_info.m_pointCount)) {
		throw std::runtime_error("not enough mem");
	}
	m_scalarFieldMap = createScalarFieldMap(table.layout()->dims(), m_info.m_pointCount);
}

void PDALToCC::done(pdal::PointTableRef table)
{
	for (const auto& item : m_scalarFieldMap)
	{
		item.second->computeMinAndMax();
		m_cloud->addScalarField(item.second);
	}
}

pdal::PointViewSet PDALToCC::run(pdal::PointViewPtr view)
{
	if (!m_cloud)
	{
		return {};
	}

	if (!m_cloud->reserve(view->size()))
	{
		return {};
	}

	for (size_t i = 0; i < view->size(); ++i)
	{
		pdal::PointRef point = view->point(i);
		processOne(point);
	}
	return {};
}
#pragma once

#include <pdal/Filter.hpp>
#include <pdal/Stage.hpp>
#include <pdal/Options.hpp>
#include <pdal/Streamable.hpp>

#include <memory>
#include <map>

#include <ccPointCloud.h>
#include <ccProgressDialog.h>

using ScalarFieldMap = std::map<pdal::Dimension::Id, ccScalarField*>;

using ProgressCallBack = std::function<void()>;
using DetermineShiftCallBack = std::function<void(const CCVector3d& point, ccPointCloud *cloud) >;

class PDALToCC : public pdal::Streamable
{
public:
	PDALToCC(pdal::QuickInfo  info, ProgressCallBack  cb);
	PDALToCC() = delete;

	std::string getName() const { return "converters.pdalToCc"; };	
	std::unique_ptr<ccPointCloud> getReadCloud()
	{
		auto cloudToReturn = std::make_unique<ccPointCloud>(nullptr);
		m_cloud.swap(cloudToReturn);
		return cloudToReturn;
	}
	void setDetermineShift(const DetermineShiftCallBack& f)
	{
		m_determineShiftCallback = f;
	}


protected:
	bool processOne(pdal::PointRef& e) override;

private:
	void prepared(pdal::PointTableRef table) override;
	pdal::PointViewSet run(pdal::PointViewPtr view) override;
	void done(pdal::PointTableRef table) override;

private:
	std::unique_ptr<ccPointCloud> m_cloud = std::make_unique<ccPointCloud>("pc");
	DetermineShiftCallBack m_determineShiftCallback = [](const CCVector3d& point, ccPointCloud *cloud){};
	ProgressCallBack m_progressCallback;
	pdal::QuickInfo m_info;
	bool m_isFirstPoint = true;
	ScalarFieldMap m_scalarFieldMap;
};


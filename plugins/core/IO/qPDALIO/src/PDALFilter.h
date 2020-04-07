#ifndef QPDAL_IO_FILTER_HEADER
#define QPDAL_IO_FILTER_HEADER

#include <pdal/StageFactory.hpp>


#include "FileIOFilter.h"

class PDALFilter : public FileIOFilter
{
public:
    PDALFilter();

	CC_FILE_ERROR loadFile(const QString& filename, ccHObject& container, LoadParameters& parameters) override;
	CC_FILE_ERROR saveToFile(ccHObject* entity, const QString& filename, const SaveParameters& parameters) override;

	bool canSave(CC_CLASS_ENUM type, bool& multiple, bool& exclusive) const override;

private:
	pdal::StageFactory m_factory;
};


#endif


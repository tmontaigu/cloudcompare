#include <ccLog.h>
#include <ccProgressDialog.h>
#include <ccGlobalShiftManager.h>

#include <QFuture>
#include <QtConcurrent>

#include <pdal/io/BufferReader.hpp>

#include "PDALConverters.h"
#include "PDALFilter.h"
#include "ccPointCloud.h"




CC_FILE_ERROR PDALFilter::loadFile(const QString& filename, ccHObject& container, FileIOFilter::LoadParameters& parameters)
{
	const std::string stdFilename = filename.toStdString();
	const std::string readerName = m_factory.inferReaderDriver(stdFilename);
    pdal::Stage *quickInfoReader = m_factory.createStage(readerName);

    ccLog::Print(QString("[PDALFilter] File: %1 driver is: %2, is driver streamable? : %3")
                .arg(filename)
                .arg(readerName.c_str())
                .arg(QString::number(quickInfoReader->pipelineStreamable())));


    pdal::Options opts;
	opts.add("filename", stdFilename);
	quickInfoReader->addOptions(opts);
	
	pdal::QuickInfo info = quickInfoReader->preview();

    auto determineCoordinateShift = [&parameters](const CCVector3d& point, ccPointCloud *cloud)
	{
		//backup input global parameters
		ccGlobalShiftManager::Mode csModeBackup = parameters.shiftHandlingMode;
		CCVector3d shiftToUse;
		bool preserveCoordinateShift = true;

		if (HandleGlobalShift(point, shiftToUse, preserveCoordinateShift, parameters))
		{
			if (preserveCoordinateShift)
			{
				cloud->setGlobalShift(shiftToUse);
			}
			ccLog::Warning("[LAS] Cloud has been recentered! Translation: (%.2f ; %.2f ; %.2f)", shiftToUse.x, shiftToUse.y, shiftToUse.z);
		}

		ccLog::Print("[PDALFilter] %f %f %f", shiftToUse.x, shiftToUse.y, shiftToUse.z);
		//restore previous parameters
		parameters.shiftHandlingMode = csModeBackup;
	};

	
	std::unique_ptr<ccPointCloud> cloud = nullptr;
    pdal::Stage *reader = m_factory.createStage(readerName);
    reader->addOptions(opts);

    if (reader->pipelineStreamable())
	{
		auto progressDialog = std::make_unique<ccProgressDialog>(false, parameters.parentWidget);
		CCLib::NormalizedProgress nProgress(progressDialog.get(), info.m_pointCount);
		progressDialog->setMethodTitle(QObject::tr("Loading file"));
		progressDialog->setInfo(QObject::tr("Points: %L1").arg(info.m_pointCount));
		const auto progressCallback = [&nProgress]() {nProgress.oneStep(); };

		PDALToCC converter(info, progressCallback);
		converter.setDetermineShift(determineCoordinateShift);
		converter.setInput(*reader);

		ccLog::Print("[PDALFilter] Executing in streaming mode");
		pdal::FixedPointTable t(1000);
		try
		{
			converter.prepare(t);
			converter.execute(t);
		}
		catch (const std::exception& e)
		{
			ccLog::Error(QString("[PDALFilter] Error: %1").arg(e.what()));
			return CC_FERR_THIRD_PARTY_LIB_EXCEPTION;
		}
		cloud = converter.getReadCloud();
	}
	else
	{
		ccLog::Print("[PDALFilter] Executing in non-streaming mode");
		pdal::PointTable table;
		const auto prepareAndExecute = [](pdal::Stage *reader, pdal::PointTable* table) -> pdal::PointViewSet
		{
			reader->prepare(*table);
			return reader->execute(*table);
		};

		auto progressDialog = std::make_unique<ccProgressDialog>(false, parameters.parentWidget);
		progressDialog->setMethodTitle(QObject::tr("Reading File"));
		progressDialog->setInfo(QObject::tr("Please wait... reading in progress"));
		progressDialog->setRange(0, 0);
		progressDialog->setModal(true);

		QFutureWatcher<pdal::PointViewSet> readerFuture;
		try
		{
			
			QObject::connect(&readerFuture, SIGNAL(finished()), progressDialog.get(), SLOT(reset()));
			readerFuture.setFuture(QtConcurrent::run(prepareAndExecute, reader, &table));

			if (progressDialog)
			{
				progressDialog->exec();
			}
			progressDialog->start();
			readerFuture.waitForFinished();
		}
		catch (const std::exception& e)
		{
			ccLog::Error(QString("[PDALFilter] Error: %1").arg(e.what()));
			return CC_FERR_THIRD_PARTY_LIB_EXCEPTION;
		}

		pdal::PointViewSet viewSet = readerFuture.result();
		pdal::PointViewPtr view = *viewSet.begin();
		ccLog::Print("ViewSize: %d", view->size());

		progressDialog = std::make_unique<ccProgressDialog>(false, parameters.parentWidget);
		CCLib::NormalizedProgress nProgress(progressDialog.get(), info.m_pointCount);
		progressDialog->setMethodTitle(QObject::tr("Loading points into CloudCompare"));
		progressDialog->setInfo(QObject::tr("Points: %L1").arg(info.m_pointCount));
		const auto progressCallback = [&nProgress]() {nProgress.oneStep(); };

		PDALToCC converter(info, progressCallback);
		converter.setDetermineShift(determineCoordinateShift);

		pdal::BufferReader bufReader;
		bufReader.addView(view);

		converter.setInput(bufReader);
		try
		{
			progressDialog->start();
			converter.prepare(table);
			converter.execute(table);
		}
		catch (const std::exception& e)
		{
			ccLog::Error(QString("[PDALFilter] Erro: %1").arg(e.what()));
			return CC_FERR_THIRD_PARTY_LIB_EXCEPTION;
		}
		cloud = converter.getReadCloud();
	}

	// TODO handle better the point view set (is it empty? handle multiple lcouds, etc)
	if (!cloud)
	{
		//TODO more helpful msg plz
		ccLog::Print("Some Error happenned...");
		return CC_FERR_CONSOLE_ERROR;
	}
	cloud->setName(filename);
	container.addChild(cloud.release());

	return CC_FERR_NO_ERROR;
}
CC_FILE_ERROR PDALFilter::saveToFile(ccHObject* entity, const QString& filename, const FileIOFilter::SaveParameters& parameters)
{
	return CC_FERR_NOT_IMPLEMENTED;
}

bool PDALFilter::canSave(CC_CLASS_ENUM type, bool& multiple, bool& exclusive) const
{
    return type == CC_TYPES::POINT_CLOUD;
}

PDALFilter::PDALFilter()
    : FileIOFilter({
        "_PDAL Filter",
        1.0f,
        {"pcd"},
        "pcd",
        {"Point Cloud Data (*.pcd)"},
        {"Point Cloud Data (*.pcd)"},
        Import | Export
    })
{
}

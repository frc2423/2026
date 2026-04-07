from wpiutil.log import DataLogReader, DataLogRecord, StartRecordData, MetadataRecordData, DoubleLogEntry
from pathlib import Path
from glob import glob
import re
import pandas as pd
import numpy as np
import plotly.graph_objects as go
import plotly.express as px
from plotly.subplots import make_subplots   
from enum import StrEnum
from wpiutil import DataLogWriter
from typing import NamedTuple
def formatData(wideDataFrame: pd.DataFrame):
    return wideDataFrame.reset_index(names="timestamp").melt(id_vars="timestamp", value_vars=list(wideDataFrame), var_name="dataName")

class DataLogAcceptedTypes(StrEnum):
    DOUBLE = "double"
    FLOAT = "float"
    INTEGER = "int64"
    BOOL = "boolean"
    DOUBLE_ARRAY = DOUBLE + "[]"
    FLOAT_ARRAY = FLOAT + "[]"
    INTEGER_ARRAY = INTEGER + "[]"
    BOOL_ARRAY = BOOL + "[]"
    STRING = "string"
    STRING_ARRAY = "string[]"

class DataReceipt:
    ntName: str
    entry: DataLogAcceptedTypes
    type: str
    metadata: dict
    timeValueStorage: dict

    def __init__(self, startData: StartRecordData):
        self.entry = startData.entry
        self.ntName = startData.name
        self.metadata = startData.metadata
        self.type = DataLogAcceptedTypes(startData.type)
        self.timeValueStorage = {}
    
    def updateMetaData(self, metadataRecord: MetadataRecordData):
        self.metadata = metadataRecord.metadata

    def finish(self):
        parsedReturnStorage = pd.Series(self.timeValueStorage)
        return parsedReturnStorage

    def addDataLogRecord(self, record : DataLogRecord):
        decodedDataPoint = self.decodeRecord(record, self.type)
        self.timeValueStorage[record.getTimestamp()] = decodedDataPoint 

    @staticmethod
    def decodeRecord(record: DataLogRecord, entryType: DataLogAcceptedTypes):
        """
        Decode a data record into a Python value based on its type string.
        """
        match entryType:  
            case DataLogAcceptedTypes.DOUBLE:
                return record.getDouble()
            case DataLogAcceptedTypes.FLOAT:
                return record.getFloat()
            case DataLogAcceptedTypes.INTEGER:
                return record.getInteger()
            case DataLogAcceptedTypes.BOOL:
                return record.getBoolean()
            case DataLogAcceptedTypes.DOUBLE_ARRAY:
                return list(record.getDoubleArray())
            case DataLogAcceptedTypes.FLOAT_ARRAY:
                return list(record.getFloatArray())
            case DataLogAcceptedTypes.INTEGER_ARRAY:
                return list(record.getIntegerArray())
            case DataLogAcceptedTypes.BOOL_ARRAY:
                return list(record.getBooleanArray())
            case DataLogAcceptedTypes.STRING:
                return record.getString()
            case DataLogAcceptedTypes.STRING_ARRAY:
                return record.getStringArray()
            case _:
                raise BufferError(f"Unrecognized type: {entryType}")
    @staticmethod
    def encodeRecord(record: DataLogRecord, entryType: DataLogAcceptedTypes, entry: int, writer: DataLogWriter):
        """
        Decode a data record into a Python value based on its type string.
        """
        timestamp = record.getTimestamp()
        match entryType:  
            case DataLogAcceptedTypes.DOUBLE:
                writer.appendDouble(entry, record.getDouble(), timestamp)
            case DataLogAcceptedTypes.FLOAT:
                writer.appendFloat(entry, record.getFloat(), timestamp)
            case DataLogAcceptedTypes.INTEGER:
                writer.appendInteger(entry, record.getInteger(), timestamp)
            case DataLogAcceptedTypes.BOOL:
                writer.appendBoolean(entry, record.getBoolean(), timestamp)
            case DataLogAcceptedTypes.DOUBLE_ARRAY:
                writer.appendDoubleArray(entry, record.getDoubleArray(), timestamp)
            case DataLogAcceptedTypes.FLOAT_ARRAY:
                writer.appendFloatArray(entry, record.getFloatArray(), timestamp)
            case DataLogAcceptedTypes.INTEGER_ARRAY:
                writer.appendIntegerArray(entry, record.getIntegerArray(), timestamp)
            case DataLogAcceptedTypes.BOOL_ARRAY:
                writer.appendBooleanArray(entry, record.getBooleanArray(), timestamp)
            case DataLogAcceptedTypes.STRING:
                writer.appendString(entry, record.getString(), timestamp)
            case DataLogAcceptedTypes.STRING_ARRAY:
                writer.appendStringArray(entry, record.getStringArray(), timestamp)
            case _:
                raise BufferError(f"Unrecognized type: {entryType}")

regexPathsOfInterest = [
    r"(Spark-[0-9]{2,})(.*)(CurrentInput)",
]
regexPerPlot = [
    [
        regexPathsOfInterest
    ]
]

class ParsedDataLoaderInput(NamedTuple):
    logFilePath: str
    regexPathsOfInterest: list[str]

class ParsedDataLoader:
    networkPathSeries : dict[str, pd.Series]
    # writer : DataLogWriter
    # currEntry : int = -1
    # currType: str
    # currName: str
    def __init__(self, input: ParsedDataLoaderInput):
        # self.writer =  DataLogWriter("testing.wpilog")
        self.networkPathSeries = self.__getAllNetworkPathsData(input.regexPathsOfInterest, input.logFilePath)
        
    def __getAllNetworkPathsData(self, regexPathsOfInterest: list[str], logFilePath: str):
        entryToReceipt: dict[int, DataReceipt] = {}
        reader = DataLogReader(logFilePath)
        skipMode = False
        for record in reader:
            if record.isStart():
                sourceStart = record.getStartData()
                # if self.currEntry != -1:
                #     self.writer.finish(self.currEntry)
                #     self.writer.flush()
                    
                # skipMode = False
                # sourceStart = record.getStartData()
                # self.currEntry = self.writer.start(sourceStart.name, sourceStart.type, sourceStart.metadata)
                # self.currName = sourceStart.name
                # try:
                #     self.currType = DataLogAcceptedTypes(sourceStart.type)
                # except:
                #     skipMode = True
                if any(re.search(rePath, sourceStart.name) for rePath in regexPathsOfInterest):
                    entryToReceipt[sourceStart.entry] = DataReceipt(sourceStart)

            elif record.isSetMetadata():
                
                sourceMeta = record.getSetMetadataData()
                if sourceMeta.entry in entryToReceipt:
                    entryToReceipt[entry].updateMetaData(sourceMeta)
            
            # Note: Ignoring Control and Finish fields, don't think we need them
            elif record.isControl() or record.isFinish():
                pass

            else:
                if skipMode:
                    continue
                entry = record.getEntry()
                if entry in entryToReceipt:
                    entryToReceipt[entry].addDataLogRecord(record)
                # try:
                #     DataReceipt.encodeRecord(record, self.currType, self.currEntry, self.writer)
                # except Exception as e:
                #     if skipMode is not None:
                #         print(f"Error in {self.currName} because {e}")
                #     skipMode = None
                        
                
        # Post process all data points
        return {dataReceipt.ntName : dataReceipt.finish() for dataReceipt in entryToReceipt.values()}


loader = ParsedDataLoader(
    ParsedDataLoaderInput(
        logFilePath=glob(str(Path(__file__).parent / "*.wpilog"))[2],
        regexPathsOfInterest=regexPathsOfInterest
    )
)


savedData : pd.DataFrame = pd.DataFrame()
for allRegexForPlot in regexPerPlot[0]:
    for plotRegex in allRegexForPlot:
        for ntName, data in loader.networkPathSeries.items():
            if re.search(plotRegex, ntName):
                savedData[ntName] = data
savedData = savedData.sort_index().fillna(0)
savedDataSum = savedData.sum(axis = 1)
writer = DataLogWriter("testing.wpilog")
entry = writer.start("NT:/sample/current_sum", DataLogAcceptedTypes.DOUBLE, metadata="Python Addition", timestamp=int(savedDataSum.index[0]))
for time, val in savedDataSum.items():
    writer.appendDouble(entry, float(val), int(time))
writer.finish(entry, int(savedDataSum.index[-1]))
writer.flush()
writer.stop()



pass
from ctypes.wintypes import MSG
import datetime
import os
import logging
from re import S
import traceback
import csv

documentPath = None


class Log:
    def __init__(self, moduleName):
        global documentPath
        super(Log, self).__init__()
        localtime = datetime.datetime.now().strftime("%Y_%m_%d_%H_%M_%S")
        self.module = moduleName
        if documentPath == None:
            documentPath = "log/{}".format(localtime)
            self.__createDocument(documentPath)
        self.filePath = "{}/{}.txt".format(documentPath, moduleName)
        self.allLogPath = "{}/{}.txt".format(documentPath, "all_log")

    def __createDocument(self, path):
        if not os.path.exists(path):
            os.makedirs(path)

    def __write2File(self, data):
        try:
            with open(self.filePath, "a") as f, open(self.allLogPath, "a") as all_f:
                f.write(data)
                all_f.write(data)
        except IOError as e:
            print(f"写入文件时发生错误: {e}")

    def __prefix(self, level):
        localtime = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
        return "[%s][%s][%s]" % (localtime, level, self.module)

    def debug(self, *args):
        s = traceback.extract_stack()
        text = " ".join("%s" % x for x in args)
        logStr = "{}[{}] {}".format(self.__prefix("Debug"), s[-2][2], text)
        try:
            self.__write2File(logStr + "\n")
            print(logStr)
        except Exception as e:
            print(f"写入文件时发生错误: {e}")

    def error(self, *args):
        s = traceback.extract_stack()
        text = " ".join("%s" % x for x in args)
        logStr = "{}[{}] {}".format(self.__prefix("Error"), s[-2][2], text)
        try:
            self.__write2File(logStr + "\n")
            print(logStr)
        except Exception as e:
            print(f"写入文件时发生错误: {e}")

    def info(self, *args):
        s = traceback.extract_stack()
        text = " ".join("%s" % x for x in args)
        logStr = "{}[{}] {}".format(self.__prefix("Info"), s[-2][2], text)
        try:
            self.__write2File(logStr + "\n")
            print(logStr)
        except Exception as e:
            print(f"写入文件时发生错误: {e}")

    def __prefix2(self, sign):
        localtime = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
        return "[%s] %s: " % (localtime, sign)

    def output(self, s):
        s = self.__prefix2(">>") + s
        return s

    def csvWriteDate(self, file_name, fieldnames, data):
        csvPath = "{}/_{}.csv".format(documentPath, file_name)
        try:
            with open(csvPath, "a", newline="") as f:
                writer = csv.DictWriter(f, fieldnames=fieldnames)
                writer.writerow(data)
        except Exception as e:
            print(f"写入文件时发生错误: {e}")

    def csvWriteHeader(self, file_name, fieldnames):
        csvPath = "{}/_{}.csv".format(documentPath, file_name)
        try:
            with open(csvPath, "w", newline="") as f:
                writer = csv.DictWriter(f, fieldnames=fieldnames)
                writer.writeheader()
        except IOError as e:
            print(f"写入文件时发生错误: {e}")

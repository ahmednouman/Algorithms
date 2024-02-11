import numpy as np

class saveData:
    def __init__(self):
        self.rawData = []
        self.filteredData = []
        self.dtData = []
        self.speedData = []

    def writeToFile(self, *args):
        f= open("Data.txt","w+")
        print(len(self.rawData))
        extraData = len(args)
        for i in range(len(self.rawData)):
            f.write(str(self.rawData[i][0])+"\t %d \t %d \t %d \t %f \t %f" %(self.rawData[i][1], self.filteredData[i][0], self.filteredData[i][1], self.speedData[i][1], self.dtData[i]))
            for data in args:
                f.write("\t %f" %(data[i]))
            f.write("\n")
        f.close()

    def clearSaved(self):
        self.rawData = []
        self.filteredData = []
        self.dtData = []
        self.speedData = []        

    # def clearSaved(self):
    #     self.rawData = []
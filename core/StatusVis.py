from unicodedata import name
import numpy as np
import matplotlib.pyplot as plt
from copy import deepcopy
import datetime
import os

class StatusVis():
    # status 履歴を可視化
    def __init__(self, status_list, dt, name, path):
        self.status_list = status_list
        self.dt = dt
        folder_name = name+str(datetime.datetime.now().strftime("%Y%m%d%H%M"))
        path_reserve = path+ "/" +folder_name
        os.makedirs(path_reserve, exist_ok=True)    
        self.path_reserve = path_reserve

    def _get_history(self, name):
        status = self.status_list.extract_status(name)
        return status.history

    def vis_history(self, name_list):
        # 用回収
        axs = []
        fig = plt.figure()
        n  = len(name_list)
        status = self.status_list.extract_status(name_list[0])
        dim = len(status.value)
        for i in range(dim):
            axs.append(fig.add_subplot(dim, 1, i + 1))
        t = np.arange(0, len(status.history), 1) * self.dt
        output = []
        for i in range(dim):
            temp_output = []
            for k in range(n):
                val = self._get_history(name_list[k])
                dim_out = []
                for j in range(len(status.history)):
                    dim_out.append(val[j][i])
                temp_dim_out = deepcopy(dim_out)
                temp_output.append(temp_dim_out)
            output.append(temp_output)

        for i in range(dim):
            axs[i].set_xlabel("time[s]")  # x軸ラベル
            #axs[i].set_ylabel()  # y軸ラベル
            axs[i].grid()
            for k in range(n):
                axs[i].plot(t, output[i][k],  label= name_list[k])
            axs[i].legend(loc=0)  # 凡例
        fig.savefig(self.path_reserve+"/"+"_".join(name_list)+".png")
        plt.show()       
        
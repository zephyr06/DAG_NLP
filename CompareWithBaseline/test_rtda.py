from __future__ import print_function
import sys
import math
import numpy as np
import time
import threading

vec = [[0, 's', 104.9599279940594], [4, 's', 108.255924002151], [4, 'f', 120.4149199911626], [2, 's', 123.11744599719532], [2, 'f', 125.08934800280258], [1, 's', 127.9939529922558], [1, 'f', 134.89906598988455], [3, 's', 137.5006320013199], [3, 'f', 140.08098100021016], [1, 's', 143.7462570029311], [0, 'f', 144.8184799955925], [3, 's', 146.78039799036924], [3, 'f', 148.9437879936304], [0, 's', 150.90577000228222], [1, 'f', 151.8363929935731], [4, 's', 153.9141829998698], [4, 'f', 169.4674159953138], [2, 's', 171.62891798943747], [2, 'f', 177.80088000290561], [1, 's', 179.8812299966812], [0, 'f', 184.42906498967204], [3, 's', 187.11350999365095], [1, 'f', 188.27715699444525], [3, 'f', 191.26889799372293], [3, 's', 200.18338499357924], [1, 's', 200.18338499357924], [3, 'f', 202.09752800292335], [1, 'f', 205.89944400126114], [0, 's', 212.1473419974791], [4, 's', 223.1103709927993], [0, 'f', 239.74478599848226], [2, 's', 241.9505439902423], [4, 'f', 244.36321399116423], [2, 'f', 247.33892299991567], [3, 's', 250.32685599580873], [1, 's', 250.32685599580873], [3, 'f', 252.51888600178063], [1, 'f', 256.5235539950663], [3, 's', 300.13813498953823], [1, 's', 300.13813498953823], [3, 'f', 302.47294799482916], [1, 'f', 306.8732319952687], [0, 's', 312.1422190015437], [4, 's', 323.1640960002551], [4, 'f', 334.92997199937236], [2, 's', 337.0515699934913], [2, 'f', 339.68379099678714], [0, 'f', 346.65533699444495], [3, 's', 350.1565490005305], [1, 's', 350.1565490005305], [3, 'f', 352.4679709953489], [1, 'f', 356.6461750015151], [3, 's', 400.1349000027403], [1, 's', 400.1349000027403], [3, 'f', 402.3405619955156], [1, 'f', 406.68558099423535], [0, 's', 412.1389520005323], [4, 's', 423.16006099281367], [4, 'f', 435.9716000035405], [2, 's', 438.4385099983774], [2, 'f', 440.5170679965522], [0, 'f', 448.43059599224944], [3, 's', 450.9195209975587], [1, 's', 450.9195209975587], [3, 'f', 453.8538869965123], [1, 'f', 458.83004200004507], [3, 's', 500.1747369969962], [1, 's', 500.1747369969962], [3, 'f', 502.19316699076444], [1, 'f', 506.04532299621496], [0, 's', 512.1717169968178], [4, 's', 525.1388719916577], [4, 'f', 537.7205239929026], [2, 's', 539.9552729941206], [0, 'f', 540.8823439938715], [2, 'f', 544.9942760023987], [3, 's', 550.2483829914127], [1, 's', 550.2483829914127], [3, 'f', 551.9324459892232], [1, 'f', 555.9471619926626], [3, 's', 600.1554379909066], [1, 's', 600.1554379909066], [3, 'f', 602.0536759897368], [1, 'f', 606.105511993519], [0, 's', 612.2044499934418], [4, 's', 626.4635719999205], [0, 'f', 643.3684509975137], [2, 's', 645.6042249919847], [4, 'f', 646.3285439967876], [2, 'f', 649.4552609947277], [3, 's', 651.2959629908437], [1, 's', 651.2959629908437], [3, 'f', 653.6844729998847], [1, 'f', 657.8270609898027], [3, 's', 700.1353069936158], [1, 's', 700.1353069936158], [3, 'f', 702.0659929985413], [1, 'f', 706.8895400007023], [0, 's', 712.1724149910733], [4, 's', 723.1560200016247], [4, 'f', 739.9152360012522], [2, 's', 745.8873900031904], [0, 'f', 747.3173399921507], [1, 's', 750.5055929941591], [2, 'f', 750.814264989458], [3, 's', 753.0197669984773], [3, 'f', 754.9536529986653], [1, 'f', 758.1978410016745], [3, 's', 800.1464720000513], [1, 's', 800.1464720000513], [3, 'f', 802.1403579914477], [1, 'f', 805.9848659904674], [0, 's', 812.1652440022444], [4, 's', 825.637806992745], [0, 'f', 836.0841639951104], [2, 's', 838.4242899919627], [4, 'f', 839.6132490015589], [2, 'f', 843.5125410032924], [3, 's', 850.1950950012542], [1, 's', 850.1950950012542], [3, 'f', 852.9740999947535], [1, 'f', 856.4950239961036], [3, 's', 900.1903089956613], [1, 's', 900.1903089956613], [3, 'f', 902.809890001663], [1, 'f', 906.7232309898827], [0, 's', 912.2031930019148], [4, 's', 923.1763659918215], [4, 'f', 936.5894729999127], [2, 's', 938.7840309937019], [2, 'f', 941.1253089929232], [0, 'f', 945.516152001801]]

class Testt:
    def __init__(self):
        self.hyperperiod = 100
        self.instances = vec
        self.chains = [[0,2,3],[1,2,3]]
    
    def calculate_max_RTDA(self):
        max_RT = []
        max_DA = []
        for chain in self.chains:
            RT = []
            DA = []
            previous_first_chain_task_start_time = -1
            previous_first_react_task_finish_time = -1
            first_chain_task_start_time = -1
            first_react_task_finish_time = -1
            for i in range(len(self.instances)):
                instance = self.instances[i]
                if (instance[0] == chain[0] and instance[1] == 's'):
                    previous_first_chain_task_start_time = first_chain_task_start_time
                    previous_first_react_task_finish_time = first_react_task_finish_time
                    first_chain_task_start_time = instance[2]
                    j = i+1
                    while not (self.instances[j][0] == chain[0] and self.instances[j][1] == 'f'):
                        j += 1
                    j += 1
                    found_first_reaction = False
                    chain_idx = 1
                    while j < len(self.instances):
                        if not (self.instances[j][0] == chain[chain_idx] and self.instances[j][1] == 's'):
                            j += 1
                        else:
                            j = j+1
                            while not (self.instances[j][0] == chain[chain_idx] and self.instances[j][1] == 'f'):
                                j += 1
                            if (chain_idx < len(chain) -1):
                                j += 1
                                chain_idx += 1
                            else:
                                first_react_task_finish_time = self.instances[j][2]
                                RT.append(first_react_task_finish_time - first_chain_task_start_time)
                                found_first_reaction = True
                                break
                    if found_first_reaction:
                        if previous_first_chain_task_start_time != -1 and previous_first_react_task_finish_time != first_react_task_finish_time:
                            j -= 1
                            while not (self.instances[j][0] == chain[-1] and self.instances[j][1] == 'f'):
                                j-=1
                            DA.append(self.instances[j][2] - previous_first_chain_task_start_time)
            max_RT.append(max(RT))      
            max_DA.append(max(DA))
        print(sum(max_RT) + sum(max_DA))
        print("max RT", max_RT)
        print("max_DA", max_DA)


if __name__ == "__main__":
    te = Testt()
    te.calculate_max_RTDA()
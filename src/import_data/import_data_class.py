# -*- coding: utf-8 -*-
#!/usr/bin/env python
import pandas as pd
from pathlib import Path
import os
import glob
from collections import OrderedDict
import re #for split
from static_configuration import *
import csv
import numpy as np
import time

DEBUG = True
IMPORT_LIDAR_MEASUREMENTS = True

class ImportedData:
    
    def find_collection(logs, reference_timestamp):
        pl = []
        sys = []
        
        if list(logs.pTS) != []:
            for exp in list(logs.pTS):
                t = np.array(logs.pTS[exp].timestamp)
                print('mean(t):', t[int(len(t)/2)], 'min_ref:', min(reference_timestamp),
                      'max_ref:', max(reference_timestamp))
                
                if min(reference_timestamp) < t[int(len(t)/2)] < max(reference_timestamp):
                    print('Collection available!!!', exp)
                    pl = logs.pTS[exp]    
                    sys = logs.datalog[exp]
        return pl, sys
    
    def get_desired_waypoints(file):
        fo = open(file)     
        names=fo.readline().replace('#','').replace('\n','').split(',')    
        still_desired_wp=True
        wp=[]
        while still_desired_wp:
            try:
                wp_line=[float(x) for x in fo.readline().replace('#','').replace('\n','').split(',')]
            except:
                wp_line=[]             
                         
            if len(wp_line) == len(names):
                wp.append(wp_line)
            else:
                still_desired_wp=False
        df = None        
        if len(wp) > 0:
            df = pd.DataFrame(np.array(wp),columns=names)
        return df

    def import_filtered_scan(self, path, txt=''):
        _angle=[]
        ts=[]
        lm=[]
        if DEBUG:
            print(path)
        if os.path.isfile(path) == False:
            path = os.path.normpath(str(path)+'.txt')
        if os.path.isfile(path) == True:                        
            # Parsing lidar measurements
            try:
                df = open(os.path.normpath(str(path)+'_angle'))
                lines = df.read().splitlines()  
                c = lines[0].split(",")
                _angle = np.arange(float(c[0]), float(c[1]), float(c[2]))
                
                lidar_fp = path
                df = open(lidar_fp)
                content = df.read().splitlines()
                ts = []
                lm = []
                
                for i in range(0,len(content)):
                    fields = content[i].split(",")
                    ts.append(int(fields[0]))
                    _lm0 = fields[1::]
                    _lm = [float(l) for l in _lm0]
                    lm.append(_lm)
                if DEBUG:
                    print("\nsize ts: " + str(len(ts)) + "\tlm: " + str(len(lm)) + "/" + str(len(lm[0])))                
            except:
                print("Empty " + txt + " filtered scan file?" + str(path))   
        return _angle, OrderedDict(zip(ts,lm))
    
    def import_general_log (path, n_lines_header):
        print('import_general_log', path)
        df = []
        if os.path.isfile(path):
    #        try:
            fo = open(path)
            n_hashtag_headers = 0
            hashtag_is_present = True
            while hashtag_is_present:
                l = fo.readline()
                if l[0] is '#':
                    n_hashtag_headers +=1
                else:
                    hashtag_is_present = False 
                
            if n_hashtag_headers > 0:
                n_lines_header = n_hashtag_headers
            df = pd.read_csv(path,  sep=",", header=n_lines_header)
    #        except:
    #            print('Check ' + path + ' file!')
        return df  
    def import_lidar_log (self, path):
        ts=[]
        lm=[]
        if DEBUG:
            print('import_lidar_log', path)
        if os.path.isfile(path) == False:
            path = os.path.normpath(str(path)+'.txt')
        if os.path.isfile(path) == True:
            try:
                df = open(path)
                content = df.read().splitlines()
                ts = []
                lm = []
                
                for i in range(0,len(content)):
                    fields = content[i].split(",")
                    ts.append(int(fields[0]))
                    _lm0 = fields[0::]
                    _lm = [float(l) for l in _lm0]
                    lm.append(_lm)
                if DEBUG:
                    print("size ts: " + str(len(ts)) + "\tlm: " + str(len(lm)) + "/" + str(len(lm[0])))
            except:
                print("Empty scan file?" + str(path))

        return OrderedDict(zip(ts,lm))

    def list (self):
        return self.__dict__.keys()

    def import_perception_lidar_log(self, pTS_fp, exp):
        if os.path.isfile(pTS_fp) == 0:
            pTS_fp = os.path.normpath(str(pTS_fp)+'.txt');                
        try:
            aux = {}
            f = open(pTS_fp)
            lines = f.readlines()
            a = re.split(',|\n',lines[0])
            for i in range(0, len(a)-1,2):
                aux[a[i]] = float(a[i+1])
            self.pTS_configs[os.path.basename(str(exp))] = aux 
            #df = pd.read_table(pTS_fp,  sep=",", header=1)
            df = pd.read_csv(pTS_fp,  sep=",", header=1)
            self.pTS[os.path.basename(str(exp))] = df  
        except:
            print("Empty file?" + str(pTS_fp)) 
                
    def __init__(self, data_dir, import_lidar=True):
        
        for log in list(specific_logs):
            vars(self)[log] = OrderedDict()

        for log in list(general_logs):
            vars(self)[general_logs[log][0]] = OrderedDict()
            
        for log in list(cam_logs):
            vars(self)[cam_logs[log]] = OrderedDict()
            vars(self)[cam_filepath[log]] = OrderedDict()      
        
        for log in list(xy_logs):
            vars(self)[xy_logs[log]] = OrderedDict()
        
        exps_dir = [x for x in Path(data_dir).iterdir() if x.is_dir()]
    
        for exp in iter(exps_dir):
            # for files directly inside an exp folder
            aux_file = os.path.join(str(exp),'name.txt')
            if os.path.isfile(aux_file):        
                fp = open(str(aux_file),'r')
                self.name[os.path.basename(str(exp))] = fp.readline()
                fp.close()
                
            ## for files inside folders in an exp folder
            for sub in exp.iterdir():
                if DEBUG:
                    print("\tsub: " + str(sub) + " lidar: " + os.path.basename(str(sub)) + \
                          " last mod time: " + str(os.path.getmtime(str(sub))))                 
                if(os.path.basename(str(sub)) == bags_folder):  
                    bags_fn = os.listdir(str(sub))            
                    for bag_fn in bags_fn:
                        if str(bag_fn).endswith('.txt'):
                            aux_fp=os.path.join(str(sub),bag_fn)
                            aux_dict_name=str(bag_fn)[:-4]
                            if DEBUG:
                                print(aux_dict_name)
                            #df = pd.read_table(aux_fp,  sep=",", header=0)  
                            df = pd.read_csv(aux_fp,  sep=",", header=0)                        
                            self.bags_aux[ aux_dict_name ]  =  df
                    self.bags[os.path.basename(str(exp))]= self.bags_aux
                for vid in list(cam_logs):
                    if(os.path.basename(str(sub)) == vid):
                        cam_fns = os.listdir(str(sub))
                        for cam_fn in cam_fns:
                            if str(cam_fn).endswith('.avi') or str(cam_fn).endswith('.mp4') :                        
                                cam_video_fp = os.path.join(str(sub),cam_fn)
                            elif str(cam_fn).endswith('.txt'):                        
                                aux_fp = os.path.join(str(sub),cam_fn)
                        
                        try:
                            if DEBUG:
                                print(vid + ': ' + str(sub) + ' ' + str(cam_fns) + ' cam_video_fp: ' + str(cam_video_fp) + ' os.path.basename(str(exp)): ' + str(os.path.basename(str(exp))) )                
                            df = pd.read_csv(aux_fp,sep=",",skiprows=1,names=cam_dnames)                            
                            vars(self)[cam_logs[vid]].update({os.path.basename(str(exp)):df})
                            vars(self)[cam_filepath[vid]].update({os.path.basename(str(exp)):cam_video_fp})
                        except:
                            print('Check ' + vid + '!')
                    
                # Parsing system_log files        
                if(os.path.basename(str(sub)) == system_folder):
                    sys_fp = os.path.join(str(sub), system_fn)
                    if os.path.isfile(sys_fp):
                        try:                  
                            t0 = time.time()          
                            #df = pd.read_table(sys_fp,sep=",",skiprows=2,header=None,names=sys_dnames, dtype={'drive_mode':np.str})  
                            df = pd.read_csv(sys_fp,sep=",", index_col=False)                  
                            vars(self)['datalog'].update({os.path.basename(str(exp)):df})
                            #self.datalog[os.path.basename(str(exp))] = df
                            print('ellapsed time to import datalog: %.6fs' % (time.time()-t0))
                        except:
                            print("Empty datalog file?")    
                if(os.path.basename(str(sub)) == lidar_folder):                     
                    for log in list(general_logs):
                        data_fp = os.path.join(str(sub), log)
                        if os.path.isfile(data_fp):
                            try:
                                #df = pd.read_table(data_fp,  sep=",", header=general_logs[log][1])
                                df = pd.read_csv(data_fp,  sep=",", header=general_logs[log][1])
                                vars(self)[general_logs[log][0]].update({os.path.basename(str(exp)):df})
                            except:
                                print('Check ' + log + ' file!')                                    
                            
                    for log in list(xy_logs):
                        aux_fp = os.path.join(str(sub), log)                            
                        if os.path.isfile(aux_fp):
                            if DEBUG:
                                print(aux_fp)
                            df = open(aux_fp)
                            content = df.read().splitlines()
                            ts = []
                            x = []
                            y = []
                            
                            for i in range(1,len(content)):
                                fields = content[i].split(",")
                                ts.append(int(fields[0]))
                                _x0 = fields[1::2]
                                _y0 = fields[2::2]
                                _x = [float(x) for x in _x0]
                                _y = [float(y) for y in _y0]
                                x.append(_x)
                                y.append(_y)
                            vars(self)[xy_logs[log]].update({os.path.basename(str(exp)): OrderedDict(zip(ts,zip(x,y)))})
                             
                    if IMPORT_LIDAR_MEASUREMENTS and import_lidar:

                        # Parsing lidar measurements
                        self.lidar[os.path.basename(str(exp))] = self.import_lidar_log(os.path.join(str(sub), lidar_fn))

                        self.secondary_lidar[os.path.basename(str(exp))] = self.import_lidar_log(os.path.join(str(sub), secondary_lidar_fn))

                        aux_fp = os.path.join(str(sub), left_filteredScan_fn)
                        self.left_lidar_angle, self.left_lidar[os.path.basename(str(exp))] = self.import_filtered_scan(aux_fp, 'left') 

                        aux_fp = os.path.join(str(sub), right_filteredScan_fn)
                        self.right_lidar_angle, self.right_lidar[os.path.basename(str(exp))] = self.import_filtered_scan(aux_fp, 'right') 
                        

                    else:
                        print('IMPORT_LIDAR_MEASUREMENTS DISABLED') 
                                      
                    # Parsing perception lidar variables saved as 'perception_lidar_log' files [TS### standard]
                    pTS_fp = os.path.join(str(sub), pTS_fn)
                    self.import_perception_lidar_log(pTS_fp, exp)
                                         
                                 
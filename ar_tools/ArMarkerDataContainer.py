import numpy as np
import pickle
from collections import namedtuple

class FramedMatrix:
    def __init__(self, parent_frame_id, M):
        self.parent_frame_id = parent_frame_id
        self.M = M

class ArMarkerDataContainer:
    def __init__(self, filename=None):
        self.data = {}
        
        if filename is not None:
            entries_ = pickle.load(open(filename, 'rb'))
            for entry in entries_:
                if entry[0] not in self.data.keys(): self.data[entry[0]] = {}
                self.data[entry[0]][entry[1]] = FramedMatrix(entry[2], np.fromstring(entry[3]).reshape([4,4]))
                
    def __len__(self):
        return len(self.data.keys())
        
    def save(self, filename, protocol=pickle.HIGHEST_PROTOCOL):
        entries_ = []
        for k,v in self.data.items():
            for frame_id,fm in v.items():
                entries_.append([ k, frame_id, fm.parent_frame_id, fm.M.tostring() ])
                
        pickle.dump(entries_, open(filename, 'wb'), protocol)
        
    def append(self, key, frame_ids_and_fms):
        if len(frame_ids_and_fms) == 0: return
    
        if key not in self.data.keys(): self.data[key] = {}
        
        for ffm in frame_ids_and_fms:
            self.data[key][ffm[0]] = ffm[1]
        
    def downsample(self, n):
        sorted_keys_ = sorted(list(self.data.keys()))
        
        skip_ = int(len(sorted_keys_) / n)
        if skip_ > 1:
            data_ = {}
            for key in sorted_keys_[::skip_]:
                data_[key] = self.data[key]
                                
            self.data = data_
        
    def generator(self):
        for k,v in self.data.items(): yield k,v

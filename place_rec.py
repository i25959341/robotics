#!/usr/bin/env python
# By Jacek Zienkiewicz and Andrew Davison, Imperial College London, 2014
# Based on original C code by Adrien Angeli, 2009

import random
import os
import time

# More init #

from particleDataStructures import *

can = Canvas()
mymap = Map(can);
particle = Particles(can)

mymap.add_wall((0,0,0,168));        # a
mymap.add_wall((0,168,84,168));     # b
mymap.add_wall((84,126,84,210));    # c
mymap.add_wall((84,210,168,210));   # d
mymap.add_wall((168,210,168,84));   # e
mymap.add_wall((168,84,210,84));    # f
mymap.add_wall((210,84,210,0));     # g
mymap.add_wall((210,0,0,0));        # h

mymap.draw();

time.sleep(3)
# End init#


PI = 3.1416
NO_BINS = 72
NO_BINS_HIST = 100
MAX_MSRT = 250 # the ma# the maximum length up to which sonar measurements are reliable 


# Location signature class: stores a signature characterizing one location
class LocationSignature:
    def __init__(self, no_bins = NO_BINS):   # s: changed from 360 to 72
        self.sig = [0] * no_bins
        
    def print_signature(self):
        for i in range(len(self.sig)):
            print self.sig[i]

# --------------------- File management class ---------------
class SignatureContainer():
    def __init__(self, size = 5):
        self.no_bins = NO_BINS;
        self.size      = size; # max number of signatures that can be stored
        self.filenames = [];
        
        # Fills the filenames variable with names like loc_%%.dat 
        # where %% are 2 digits (00, 01, 02...) indicating the location number. 
        for i in range(self.size):
            self.filenames.append('loc_{0:02d}.dat'.format(i))

    # Get the index of a filename for the new signature. If all filenames are 
    # used, it returns -1;
    def get_free_index(self):
        n = 0
        while n < self.size:
            if (os.path.isfile(self.filenames[n]) == False):
                break
            n += 1
            
        if (n >= self.size):
            return -1;
        else:    
            return n;

    # Delete all loc_%%.dat files
    def delete_loc_files(self):
        print "STATUS:  All signature files removed."
        for n in range(self.size):
            if os.path.isfile(self.filenames[n]):
                os.remove(self.filenames[n])
            
    # Writes the signature to the file identified by index (e.g, if index is 1
    # it will be file loc_01.dat). If file already exists, it will be replaced.
    def save(self, signature, index):
        filename = self.filenames[index]
        if os.path.isfile(filename):
            os.remove(filename)
            
        f = open(filename, 'w')

        for i in range(len(signature.sig)):
            s = str(signature.sig[i]) + "\n"
            f.write(s)
        f.close();

    # Read signature file identified by index. If the file doesn't exist
    # it returns an empty signature.
    def read(self, index):
        ls = LocationSignature()
        filename = self.filenames[index]
        if os.path.isfile(filename):
            f = open(filename, 'r')
            for i in range(len(ls.sig)):
                s = f.readline()
                if (s != ''):
                    ls.sig[i] = int(s)
            f.close();
        else:
            print "WARNING: Signature does not exist."
        
        return ls
    
    
####################################################################################

# s: DONE (spins sonar to capture a signature and store it in ls)
def characterize_location():
    # print "TODO:    You should implement the function that captures a signature."
    rotation = 360.0/NO_BINS

    new_sig = []
    # for each of the NO_BINS bins
    for measure in range(NO_BINS):
        print 'making measurement' 
        # make measurement 
        depth = controller.getSonarReading()
        #time.sleep(1)
        # rotate sonar
        controller.turnSonar(rotation) 
        #time.sleep(1)
        # save measurement 
        new_sig.append(depth)
    # copy this to a signature object
    ls = LocationSignature()
    ls.sig = new_sig
            
    # get the sonar back to original position
    controller.turnSonar(-NO_BINS*rotation) #-rotations*NO_BINS)
    
    #print the signature
    ls.print_signature()
    
    # return the newly created signature
    return ls

    
# s: DONE
def compare_signatures(ls1, ls2):
    dist = 0
    print "Comparing."
    for i in range(len(ls1.sig)):
        dist += (ls1.sig[i]-ls2.sig[i])**2
    return dist

# This function characterizes the current location, and stores the obtained 
# signature into the next available file.
def learn_location():
    print 'learning'
    ls =  characterize_location()
    idx = signatures.get_free_index();
    if (idx == -1): # run out of signature files
        print "\nWARNING:"
        print "No signature file is available. NOTHING NEW will be learned and stored."
        print "Please remove some loc_%%.dat files.\n"
        return
    
    signatures.save(ls,idx)
    print "STATUS:  Location " + str(idx) + " learned and saved."
    
    
# s: DONE
# This function tries to recognize the current location.
# 1.   Characterize current location
# 2.   For every learned locations
# 2.1. Read signature of learned location from file
# 2.2. Compare signature to signature coming from actual characterization
# 3.   Retain the learned location whose minimum distance with
#      actual characterization is the smallest.
# 4.   Display the index of the recognized location on the screen
def recognize_location():
    ls_obs = characterize_location();
    idx_most_similar = -1
    smallest_dist = 10000000

    for idx in range(signatures.size):
        print 'STATUS:  Comparing signature ' + str(idx) + ' with the observed signature.'
        ls_read = signatures.read(idx);
        dist    = compare_signatures(ls_obs, ls_read)
        if (dist < smallest_dist):
            idx_most_similar = idx
    
    if (idx_most_similar == -1): 
        print 'Problem in the recognizeLocation()' 
    else: 
        print 'Most similar location found is: ' + idx_most_similar

        
        
####### >> Part 4.3  (Invariant measurement & recognition) :         #######  
            
            
# This function characterizes the current location, and stores the obtained 
# HISTOGRAM into the next available file.
def learn_location_invariant():
    print 'learning'
    ls =  characterize_location_invariant()
    idx = histograms.get_free_index();
    if (idx == -1): # run out of signature files
        print "\nWARNING:"
        print "No signature file is available. NOTHING NEW will be learned and stored."
        print "Please remove some loc_%%.dat files.\n"
        return
    
    histograms.save(ls,idx)
    print "STATUS:  Location " + str(idx) + " learned and saved."
    
#s: DONE
def characterize_location_invariant():
    rotation = 360.0/NO_BINS

    new_histogram = [0] * (NO_BINS_HIST+1)

    # for each of the NO_BINS bins
    for measure in range(NO_BINS):
        # make measurement 
        depth = controller.getSonarReading()
        print 'making measurement' 
        # rotate sonar
        controller.turnSonar(rotation) 
        # save measurement in the appropriate bin of the depth histogramm
        if (depth < MAX_MSRT):
            index = int(depth/(MAX_MSRT / NO_BINS_HIST))    
            if (index > NO_BINS_HIST): index = NO_BINS_HIST
            new_histogram[index] += 1

    # get the sonar back to original position
    controller.turnSonar(-NO_BINS*rotation) #-rotations*NO_BINS)
    # copy the histogram to a signature object 
    ls = LocationSignature(NO_BINS_HIST)
    ls.sig = new_histogram
    # return the newly created signature (histogram)
    return ls
    
# s: NOT COMPLETE (SEE CAPITALIZED BIT IN PART 4)
# This function tries to recognize the current location.
# 1.   Characterize current location: histogram AND signature
# 2.   For every learned locations
# 2.1. Read histogram of learned location from file, compare with measured histogram
# 2.2. Compare signature to signature coming from actual characterization in order to determine angle of rotation
 # 4.   Display the index of the recognized location on the screen AND THE ROTATION ANGLE 
def recognize_location_invariant():
    # 1.   Characterize current location: histogram AND signature    
    sig_obs = characterize_location();
    hist_obs = characterize_location_invariant();
    idx_most_similar = -1
    smallest_dist = 10000000

    # 2.   For every learned locations
    for idx in range(signatures.size):
        # 2.1. Read histogram of learned location from file, compare with measured histogram

        hist_read = histograms.read(idx);
        dist    = compare_signatures(hist_read, hist_obs)
        if (dist < smallest_dist):
            idx_most_similar = idx
    
    if (idx_most_similar == -1): 
        print 'Problem in the recognizeLocationInvariant()' 
    else: 
        print 'Most similar location found is: ' + idx_most_similar + '. Now getting the angle.'
    
    ref_sig = signatures.read(idx_most_similar)
    angle = getAngle(sig_obs, ref_sig)
    
    return [idx_most_popular, angle] 
        
        
        
        
              
# given two signatures of a same place, get the shift]
def getAngle(sig1, sig2):
    angle=-1
    score_min = -1 
    candidate = LocationSignature()

    # incrementally rotate teh observed sig to get the candidate
    for i in range(NO_BINS):
        candidate = shift(sig1,i)
        # compare candidate with saved sig
        score = compare(sig2, candidate)
        if (score<score_min):
            score = score_min
            angle = i
        
    # convert angle to degrees
    angle = angle * 360/NO_BINS
    
    return angle


# returns a shifted version of sig1
def shift(sig1, i):
    candidate = LocationSignature()
    lengthSig = len(sig1)
    for j in range(lengthSig):
        candidate[(j+i) % lengthSig] = sig1[j]
    return candidate
    
    

###############################################################################################
#
#
#                                        MAIN PROGRAM
#
#
###############################################################################################

# Prior to starting learning the locations, it should delete files from previous
# learning either manually or by calling signatures.delete_loc_files(). 
# Then, either learn a location, until all the locations are learned, or try to
# recognize one of them, if locations have already been learned.





'''
signatures = SignatureContainer(6)
histograms = SignatureContainer(6)
signatures.delete_loc_files()
histograms.delete_loc_files()

# > 4.2
# step 1 - Learn signatures (this part is to be commented out after it has been run)
for i in range(5):
    learn_location()
    time.sleep(10)     # must be put manually to next key location :/ !

# step 2 - Recognize signatures (i.e., testing previous bit):
for i in range(5):
    recognize_location();     
    time.sleep(10)     # must be put manually to next key location :/ !

# > 4.3

signatures_invariant = SignatureContainer(5);
signatures_invariant.delete_loc_files()

# step 1 -  make invariant measurement of a measurement 
for i in range(5):
    learn_location_invariant()
    time.sleep(20)     # must be put manually to next key location :/ !

# step 2 - Recognize invariant signatures (i.e., testing previous bit) + displays the shift :
for i in range(5):
    recognize_location_invariant();     
    time.sleep(20)     # must be put manually to next key location :/ !

'''

















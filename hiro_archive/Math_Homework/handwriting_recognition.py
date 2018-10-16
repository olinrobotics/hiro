#!/usr/bin/env python
import rospy
import rospkg
import time

from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError

import numpy as np
import copy
from scipy import stats
from Character import Character
from os import listdir
from os.path import isfile, join

import cv2
import img_processing as Process #Library of image processing functions in edwin/scripts/sight
import csv

# Source: http://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_ml/py_svm/py_svm_opencv/py_svm_opencv.html
class HandwritingRecognition:
    ''' DOCSTRING:
        Contains the attributes and methods to implement Edwin's handwriting
        recognition capability
        '''

    def nothing(x):
        """ DOCSTRING:
            Provides empty callback function for other functions needing a
            function for param
            """
        pass

    def __init__(self):
        ''' DOCSTRING:
            Initializes ros nodes, state vars, windows, etc. for current HR obj;
            Contains lists differentiating alpha & numeric symbols
            '''
        rospy.init_node('handwriting_recognition', anonymous=True) # makes self into node


        rospy.Subscriber('usb_cam/image_raw', Image, self.img_callback) # subscribes to cam feed

        self.bridge = CvBridge() # converts image types to use with OpenCV

        # Builds path to find picture of numbers
        rospack = rospkg.RosPack()
        self.PARAMS_PATH = rospack.get_path('edwin')
        self.img = cv2.imread(self.PARAMS_PATH + '/params/test_imgs/digits.png')

        self.pub = rospy.Publisher('word_publish',String,queue_size=10) # publishes found words

        self.detect = True
        self.frame = None

        #for looping
        self.is_running = False

        # Builds window to view and control output
        cv2.namedWindow('image')
        # cv2.createTrackbar('X','image',0,255,self.nothing)
        # cv2.setTrackbarPos('X','image',255)
        # cv2.createTrackbar('Y','image',0,255,self.nothing)
        # cv2.setTrackbarPos('Y','image',7)
        # print os.getcwd()

        # Define vars for keeping track of new words
        self.last_word = ''
        self.last_time = time.time()
        self.curr_data = ''
        self.found_word = False

        self.is_running = True # boolean representing whether or not class is reading

        # Lists of alpha symbols & numeric symbols for classify_writing method
        alpha_symb = "AaBbCcDdEeFfGgHhIiJjKkLMmNnPpQqRrTtUuVvWwYyZz"
        self.alpha_symb = list(alpha_symb)
        numer_symb = "2346789+-/*=)("
        self.numer_symb = list(numer_symb)

    def img_callback(self, data):
        ''' DOCSTRING:
            Given img data from usb cam, saves img for later use; called every
            time img recieved from usb cam
            '''
        try:
            # Saves image; converts to opencv format
            self.curr_frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print('ERR:%d',e)

    def process_data_svm(self):
        '''DOCSTRING:
            Saves .npz files of training data & training_labels for SVMs
            (alphabetic, numeric, symbolic (both alpha and num))
            '''

        path =  self.PARAMS_PATH + '/params/char_data/'
        files = listdir(path) # Lists files in path folder (training data)

        # Builds alphabetic character data list
        alpha_files = ['1','a','b','c','d','e','f','g','h','j','k','m','n','o','p','q','r','s','t','u','v','w',
                        'y','z','dot','mlt']
        alpha_files = [alphafile + '.png' for alphafile in alpha_files]

        # Builds numeric character data list
        num_files = ['0','1','2','3','4','5','6','7','8','9','mns','pls','div','dot','a','lpr','crt','rpr','mlt']
        num_files.extend(self.collect_mult_datafiles(num_files, path))
        num_files = [numfile + '.png' for numfile in num_files]
        # Builds training data & labels for alphabetic, numeric, and both; saves
        # data as .npz files
        train_data, responses = self.build_train_data(files,path)
        train_alphadata, alpha_responses = self.build_train_data(alpha_files,path)
        train_numdata, num_responses = self.build_train_data(num_files,path)
        np.savez(self.PARAMS_PATH + '/params/svm_data.npz',train=train_data,train_labels=responses) # Saves training data and labels into .npz file
        np.savez(self.PARAMS_PATH + '/params/svm_alphadata.npz',train=train_alphadata,train_labels=alpha_responses) # Saves alphabetic training data & labels into .npz file
        np.savez(self.PARAMS_PATH + '/params/svm_numdata.npz',train=train_numdata,train_labels=num_responses) # Saves numeric training data & labels into .npz file

    def collect_mult_datafiles(self, chars, file_path):
        """ DOCSTRING:
            Given list of chars & file path, looks for files in filepath w/
            format '[char]_[number]' (used for multiple data files of same char)
            """
        files = []
        for char in chars:
            incr = 1 # loop counter
            while True:
                datafile = file_path + char + '_' + str(incr) + '.png'
                if isfile(datafile):
                    files.append(char + '_' + str(incr))
                    incr += 1
                else: break

        return files


    def train_svm(self,file_name):
        '''DOCSTRING:
            Given .npz file of training data and labels, initializes, sets
            parameters/data for, and trains SVM to distinguish between chars in
            provided test data; returns SVM
            '''

        # Sets parameters of svm to use
        svm_params = dict(kernel_type = cv2.SVM_LINEAR, svm_type = \
                            cv2.SVM_NU_SVC, nu=.105)
                            # gamma = 5.383

        # reads data in .svm file and formats for svm
        with np.load(self.PARAMS_PATH + '/params/' + file_name + '.npz') as input_data:
            print('MSG: training data length: ' + str(len(  input_data['train'])))
            print('MSG: training data length: ' + str(len(input_data['train_labels'])))
            train_data = input_data['train']
            data_labels = input_data['train_labels']

        SVM = cv2.SVM()
        SVM.train(train_data,data_labels,params=svm_params)
        return SVM

    def build_train_data(self,files,path):
        """ DOCSTRING:
            Given list of test data files & path to files; returns train_data &
            labels for training SVM
            """

        # Inits data and responses to empty numpy arrays (so as to concatenate w/ other numpy arrays)
        train_data = np.float32(np.zeros((0,64)))
        responses = np.float32(np.zeros((0,1)))

        for f in files: # Loads the .pngs for the training data for each symbol
            file_path = path + f
            code = f.split('.',1)[0]
            #try:
            train_img = cv2.imread(file_path)
            print('MSG: loaded: ' + file_path)

            # Converts training data into usable format
            gray = cv2.cvtColor(train_img,cv2.COLOR_BGR2GRAY)
            width, height = gray.shape[:2]
            print('MSG: width is :' + str(width/20) + ' height is :' + str(height/20))
            cells = [np.hsplit(row,width/20) for row in np.vsplit(gray,height/20)]
            # deskewed = [map(self.deskew,row) for row in cells]
            hogdata = [map(Process.hog,row) for row in cells]

            # Builds training data
            train_data = np.concatenate((train_data, np.float32(hogdata).reshape(-1,64)), axis=0)
            # Builds labels for training data
            # http://stackoverflow.com/questions/29241056/the-use-of-python-numpy-newaxis
            responses = np.concatenate((responses,np.float32(np.repeat([self.decode_file(code)],width/20 * height/20)[:,np.newaxis])),axis=0)
            #except:
            #    print('ERR: file ' + f + ' does not exist at ' + file_path)

        return train_data, responses

    def decode_file(self, code, classification=0):
        ''' DOCSTRING:
            Given file name to decode, returns char to which file name
            corresponds
            '''
        if (code == 'dvd'): return ord('/')
        elif (code == 'div'): return ord('\\')
        elif (code == 'pls'): return ord('+')
        elif (code == 'mns'): return ord('-')
        elif (code == 'dot'): return ord('.')
        elif (code == 'crt'): return ord('^')
        elif (code == 'lpr'): return ord('(')
        elif (code == 'rpr'): return ord(')')
        elif (code == 'two'): return ord('2')
        elif (code == 'chk'): return ord('_')
        elif (len(code) == 5):
            if (code[3] == '_'):
                print('MSG: ' + code + ' = ' + code[0:3])
                return self.decode_file(code[0:3])
        elif( len(code) == 3):
            if (code[1] == '_'):
                print('MSG: ' + code + ' = ' + code[0])
                return ord(code[0])
        elif (len(code) == 1):
            return ord(code)
        if classification == 1 or classification == 0:
            if (code == 'mlt'): return ord('x')
        elif classification == 2:
            if (code == 'mlt'): return ord('*')
        else:
            print('ERR: no symbol exists for this code!')

    def SVM_predict(self,svm,data,chars):
        """ DOCSTRING:
            Given SVM & data set, returns resulting predicted data from SVM
            """

        results = svm.predict_all(data)
        res_data = []
        for x in results:
            res_data.append(int(x.item(0)))
        for idx,roi in enumerate(chars):
            roi.result = res_data[idx]

        return res_data

    def SVM_build_chars(self, svm, data, chars):
        """ DOCSTRING:
            Given SVM and data from which to build prediction, edits chars
            to reflect the current SVM & data set
            """
        res_data = self.SVM_predict(svm, data, chars)

        # Resolves fuzzy contours (i,j,=) (multi-contour characters)
        lines = [val for val in chars if chr(val.result) == '1']
        dots = [res for res in chars if chr(res.result) == '.']
        dashes = [obj for obj in chars if chr(obj.result) == '-']
        chars[:] = [val for val in chars if chr(val.result) != '1' \
            and chr(val.result) != '.' and chr(val.result) != '-']
        fuzzy_conts = self.resolve_symbols(dots,lines,dashes)

        if len(fuzzy_conts) > 0:
            chars.extend(fuzzy_conts)
        return chars

    def process_digits(self,test_data,detect_words = False):
        '''DOCSTRING
            Given raw contour data from an image and boolean representing
            whether or not to detect words, returns a 20x20 list of ROIs
            corresponding to digits found in the image'''

        if len(test_data) != 0:
            # Prepares input data for processing
            reshape_data = np.float32([char.HOG for char in test_data]).reshape(-1,64)
            pass1_chars = copy.deepcopy(test_data)

            # Predicts symbols from reshaped data
            #pass1_chars = self.SVM_build_chars(self.SVM, reshape_data, pass1_chars)
            self.chars = self.SVM_build_chars(self.SVM, reshape_data, pass1_chars)
            # Check if sentence or equation, apply corresponding SVM
            # classification = self.classify_writing(pass1_chars)
            # if classification == 1:
            #     self.SVM_build_chars(self.SVM_alpha,reshape_data, self.chars)
            # elif classification == 2:
            #     self.SVM_build_chars(self.SVM_num,reshape_data, self.chars)

            # Disp characters on screen in location corresponding to the image
            if self.frame is not None:
                for roi in self.chars:
                    cv2.circle(self.frame,(roi.x,roi.y), 5, (0,0,255), -1)
                    try:
                        cv2.putText(self.frame,chr(int(roi.result)),(roi.x,roi.y+roi.h) \
                                ,cv2.FONT_HERSHEY_SIMPLEX, 4,(0,255,0))
                    except:
                        print('ERR: tried to draw char (address ' + str(int(roi.result)) + ') that does not exist')

            # TODO: word detection
            if detect_words:
                self.detect_new_word(self.chars)
            else:
                return test_data

    def resolve_symbols(self, dot_contours,line_contours,dash_contours): # Currently sorts: i,!,l,=,/,
        '''DOCSTRING
            DESC: resolves "fuzzy" symbols that are built out of smaller symbols
            ARGS:
            self - HandwritingRecognition object - self-referential
            dot_contours - list - list of contours classified as .
            line_contours - list - list of contours classified as |
            dash_contours - list - list of contours classified as -
            RTRN: dict of contours and corresponding ASCII values
            '''

        final_contours = []

        # Goes through vertical lines to differentiate between (i,l,1)
        for line in line_contours:

            # Check for 'i' and '!' by checking each dot pos rel to line
            for dot in dot_contours:

                # If dot is in line along y and reasonably close in x-dir
                if abs(line.x - dot.x) < 50 and (0 < line.y - dot.y < 150): #TODO: make limits on spacing scale with image size
                    line.result = ord('i')
                    dot.h += line.h
                    dot.y = line.y

                    # Remove the dot-line pair from lists left to sort
                    line_contours[:] = [val for val in line_contours if val is not line]
                    dot_contours[:] = [val for val in dot_contours if val is not dot]
                    final_contours.append(line)
                    break

                # If dot is in line along y and reasonably close in x-dir
                if abs(line.x - dot.x) < 50 and (0 < dot.y - line.y < 150): #TODO: make limits on spacing scale with image size
                    line.result = ord('!')
                    dot.h -= line.h
                    dot.y = line.y

                    # Remove the dot-line pair from lists left to sort
                    line_contours[:] = [val for val in line_contours if val is not line]
                    dot_contours[:] = [val for val in dot_contours if val is not dot]
                    final_contours.append(line)
                    break

            #TODO: check adjacent contours to see if math or letter to diff 1 and l (need way to get adj. contours)

        # Goes through dashes to differentiate between (/,=,)
        for dash in dash_contours:

            # Check for '=' by checking each dash pos rel to dash
            for dash_2 in dash_contours:

                # If dash is in line along y and reasonably close in x-dir:
                if (abs(dash.x - dash_2.x) < 50) and (0 < dash_2.y-dash.y<150):
                    dash_2.result = ord('=')

                    # Remove dash-dash pair from lists left to sort
                    dash_contours[:] = [val for val in dash_contours if (val is not dash and val is not dash_2)]
                    final_contours.append(dash_2)
                    break

            # Check for '/' by checking each dot pos rel to dash
            for dot in dot_contours: # Check for top dot

                # If dot is in line along y and reasonably close in x-dir:
                if (abs(dash.x - dot.x) < 50) and (0 < dash.y - dot.y < 150):

                    for dot2 in dot_contours: # Check for bottom dot

                        if (abs(dash.x - dot2.x) < 50) and (0 < dot2.y - dash.y < 150):
                            dash.result = ord('/')
                            dot.h += dash.h
                            dot2.h -= dash.h
                            dot.y = dash.y
                            dot2.y = dash.y

                            # remove dash-dot-dot triple from lists left to sort
                            dash_contours[:] = [val for val in dash_contours if val is not dash]
                            dot_contours[:] = [val for val in dot_contours if (val is not dot and val is not dot2)]
                            final_contours.append(dash)
                            break

        # If no special chars, set reg chars
        for dash in dash_contours:
            dash.result = ord('-')
            final_contours.append(dash)
        for dot in dot_contours:
            dot.result = ord('.')
            final_contours.append(dot)
        for line in line_contours:
            line.result = ord('1')
            final_contours.append(line)

        return final_contours

    def classify_writing(self, chars):
        """ DOCSTRING:
            Given set of chars to interpret, returns list written alphabetically
            or numerically dependent on the ratio of alpha to numeric chars
            present, as determined by lists in HR object init() method

            Classification: 0 - null, 1 - alphabetic, 2 - numeric
            """

        if len(chars) > 0:
            classify = 0
            chars = [chr(int(roi.result)) for roi in chars]
            alpha_chars = [char for char in chars if char in self.alpha_symb] # list of chars classified as alphabetic
            numer_chars = [char for char in chars if char in self.numer_symb] # list of chars classified as numeric

            if len(chars) == 0:
                classify = 0
            elif len(alpha_chars) == 0:
                classify = 2
            elif len(numer_chars) == 0:
                classify = 1
            else:
                ratio = len(alpha_chars)/len(numer_chars)
                if ratio < 0.5:
                    classify = 2
                else:
                    classify = 1
            return classify

    def detect_new_word(self,char_list):
        '''DOCSTRING
            Given a list of characters, sorts the list and makes sure that the
            identified characters stay consistent for two seconds, then
            publishes them
            '''

        char_list.sort(key = lambda roi: roi.x) # Sort characters by x pos
        try:
            word = ''.join([chr(int(item.result)) for item in char_list]) # Form a word
            if word == self.curr_data: # If the current and prev words match
                # The word must remain consistent for 2 seconds
                if time.time() - self.last_time > 2 and self.found_word == False:
                    self.last_word = word
                    self.found_word = True
                    self.pub.publish(word)
                    return word

            else: # A new word is found, reset the timer
                self.last_time = time.time()
                self.curr_data = word
                self.found_word = False
                return None

        except ValueError:
            print('ERR: char address in char_list passed to detect_new_word is invalid; char_list values: ' + str([item.result for item in char_list]))

    # [TODO] REFACTOR
    def find_words(self,char_list):
        '''
            DESC:
            ARGS:
            self - HR object - refers to the current object
            char_list:
            RTNS:
            '''
        # Find lines (this is terrible - refactor)
        line_space = []
        char_list.sort(key = lambda roi: roi.y)
        for ind in range(1,len(char_list)):
            next_y = char_list[ind].y
            curr_y = char_list[ind-1].y
            line_space.append(char_list[ind].y - char_list[ind-1].y)
        z_vals = stats.zscore(line_space)

        line_index = [0]
        for idx,val in enumerate(z_vals):
            if val > 1:
                line_index.append(idx+1)
        line_index.append(len(char_list))
        lines = []
        for idx in range(0,len(line_index)-1):
            lines.append(char_list[line_index[idx]:line_index[idx+1]])

        # Find words in lines(this is terrible - refactor)
        for line in lines:
            space_size = []
            line.sort(key = lambda roi: roi.x)
            for ind in range(1,len(line)):
                next_x = char_list[ind+1].x
                curr_x = char_list[ind].x + char_list[ind].h
                space_size.append(char_list[ind].x - char_list[ind-1].x)
            z_vals = stats.zscore(space_size)

            word_index = [0]
            for idx,val in enumerate(z_vals):
                if val > 1:
                    word_index.append(idx+1)
            word_index.append(len(line))

            words = []
            for idx in range(0,len(word_index)-1):
                words.append(''.join([str(x.result) for x in line[word_index[idx]:word_index[idx+1]]]))
            #print ' '.join(words)
        # print ''
        # for word in lines:
        #     print ' '.join([str(x.result) for x in word])


    def update_frame(self):
        ''' DOCSTRING:
            updates frame with the current frame; cuts out Edwin's eyelid
            '''
        self.frame = Process.get_edwin_vision(self.curr_frame)

    def output_image(self):
        '''DOCSTRING
            Displays current frame'''

        new_frame = self.frame
        cv2.imshow('image', new_frame)
        cv2.waitKey(1)

    def get_image_text(self, frame):
        '''
            DESC: Get the words out of an image
            ARGS:
                self - object - current HR object
                frame - image - current image to analyze
            RETURNS: string representing text found in frame
            '''
        self.chars = Process.get_text_roi(frame,show_window=False)
        self.chars = self.process_digits(self.chars)

        # If chars exists,
        if self.chars:
            word = get_publish_text(chars)
            return word

    def get_publish_text(self, chars):
        '''DOCSTRING:
            Given a list of Regions of Interest (roi), returns a list sorted
            to make sense mathematically or syntactically (sentences)
            '''

        chars.sort(key = lambda roi: roi.x)
        word = ''.join([chr(item.result) for item in chars])


    def run(self,rebuild):
        ''' DOCSTRING:
            given bool representing whether or not to rebuild training data sets
            runs HR initialization code and main loop code
            '''

        r = rospy.Rate(10)
        time.sleep(2)

        # If new training data, set rebuild to True to rebuild SVMs
        if rebuild == True: self.process_data_svm()
        self.SVM = self.train_svm('svm_data')
        self.SVM_alpha = self.train_svm('svm_alphadata')
        self.SVM_num = self.train_svm('svm_numdata')

        while not rospy.is_shutdown(): # MAIN LOOP
            if self.is_running:

                e1 = cv2.getTickCount()
                self.update_frame()
                # out_image = Process.get_paper_region(self.frame)
                self.chars = Process.get_text_roi(self.frame)
                self.process_digits(self.chars,detect_words=True)
                #self.find_words(self.chars)
                self.output_image()
                e2 = cv2.getTickCount()
                # print (e2-e1)/cv2.getTickFrequency()
                r.sleep()

        cv2.destroyAllWindows()

if __name__ == '__main__':
    hr = HandwritingRecognition()
    reset_SVM = None
    # Check whether or not to update SVMs
    while (reset_SVM == None):
        query = raw_input('RQS: Update SVMs? (y/n)')
        if query == 'y':
            reset_SVM = True
        elif query == 'n':
            reset_SVM = False
        else:
            print('ERR: Did not understand input, please type "y" or "n"')

    hr.run(reset_SVM)

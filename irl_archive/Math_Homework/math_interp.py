#!/usr/bin/env python
'''
math_interp.py
Purpose: input a string that is a math equation, output solution
Author: Hannah Kolano
hannah.kolano@students.olin.edu

RUN:
roscore
arm_node.py
create_routes.py
arm_behaviors.py
arm_write.py
(test_arm_pub.py with R_look or look)
roslaunch usb_cam usb_camera.launch
handwriting_recognition.py
THEN this one
'''
from __future__ import division
import rospy
# import rospkg
from std_msgs.msg import String
import math as m
from edwin.msg import *
import time

# global variables
integer_list = ['0', '1', '2', '3', '4', '5', '6', '7', '8', '9', '.']
operator_list = ['+', '-', '/', '*', '^', '(', ')', '=']
variable_list = ['d']
placeholder_list = ['p', 'q', 'r', 's', 't', 'u']


class Calculator:
    def __init__(self):
        '''initializes the object'''

        rospy.init_node('math_solving')

        self.pub = rospy.Publisher('/write_cmd', Edwin_Shape, queue_size=10)
        self.pub_behave = rospy.Publisher('/behaviors_cmd', String, queue_size=10)
        self.pub_speak = rospy.Publisher('/edwin_speech_cmd', String, queue_size=10)
        self.status_sub = rospy.Subscriber('/arm_status', String, self.status_callback, queue_size=10)
        self.writing_sub = rospy.Subscriber("/writing_status", String, self.writing_callback, queue_size=10)
        self.tree = tuple()
        self.eqn = ''
        self.status = None
        self.writing_status = 0
        rospy.Subscriber('word_publish', String, self.cmd_callback)
        time.sleep(2)
        print 'initialized'

    def writing_callback(self, data):
        command = data.data
        if command == "writing":
            self.writing_status = 0
        elif command == "done":
            self.writing_status = 1

        self.tree = tuple()
        self.eqn = ''
        rospy.Subscriber('word_publish', String, self.cmd_callback)

    def cmd_callback(self, data):
        '''callback'''
        # first several characters are not part of the equation, so removes them
        self.eqn = str(data)[6:]
        # self.pub.publish(data)

    def status_callback(self, data):
		print "arm status callback", data.data
		if data.data == "busy" or data.data == "error":
			self.status = 0
		elif data.data == "free":
			self.status = 1

    def check_completion(self):
		"""
		makes sure that actions run in order by waiting for response from service
		"""

		time.sleep(3)
		while self.status == 0:
			pass

    def solve_simple(self, eqn):
        '''takes in a simple equation; solves it; returns a string of the answer'''
        eqn = eqn[0:-1] if eqn[-1] == '=' else eqn
        # changes carets to ** because that's what python reads as exponents
        if '^' in eqn:
            eqn = eqn[:eqn.find('^')] + '**' + eqn[eqn.find('^')+1:]
        answer = eval(eqn)
        answer = "{0:.2f}".format(answer) if type(answer) == float else answer
        return str(answer)

    def fixes_letters(self, eqn):
        ''' takes in an equation;
        turns the letters into the numbers they probably represent;
        returns the parsed equation '''
        replace = {'o':'0', 'l':'1', 't':'+', 'q':'9'}
        for letter in replace.keys():
            if letter in eqn:
                index = eqn.find(letter)
                eqn = eqn[:index] + replace[letter] + eqn[index+1:]
        return eqn

    def initialize_algebra(self, eqn):
        '''takes in an equation; finds any weird errors; if it's clean, solves algebra
        does not return anything'''
        eqn = self.fixes_letters(eqn)
        if not all(digit in variable_list or digit in integer_list or digit in operator_list for digit in eqn):
            raise ValueError("I do not know that character")
            self.pub_behave.publish('sad')
            self.check_completion()
            self.pub_speak.publish("I do not know that character. Give me another problem.")
            time.sleep(3)
            self.num_demos += 1
        found_variables = []
        for variable in variable_list:
            if variable in eqn:
                self.variable, index = variable, eqn.find(variable)
                found_variables.append(variable)
                if eqn[index+1:].find(variable) != -1:
                    raise ValueError('I found two instances of the same variable')
                    self.pub_behave.publish('sad')
                    self.check_completion()
                    self.pub_speak.publish("I can't solve that. Give me another problem.")
                    time.sleep(3)
                    self.num_demos += 1
        if len(found_variables) > 1:
            raise ValueError('I found too many variables')
            self.pub_behave.publish('sad')
            self.check_completion()
            self.pub_speak.publish("I can't solve that. Give me another problem.")
            time.sleep(3)
            self.num_demos += 1
        actual_ops = operator_list[:4]
        for n in range(len(eqn)-1):
            print(eqn[n])
            if eqn[n] in actual_ops and eqn[n+1] in actual_ops and eqn[n+1] != '-':
                raise ValueError('Two operations in a row?')
                self.pub_behave.publish('sad')
                self.check_completion()
                self.pub_speak.publish("That's not how you write equations. Give me another problem.")
                time.sleep(3)
                self.num_demos += 1
        else:
            # if it's clean, initialize the tree
            eqn = self.parse_var_mul(eqn)
            self.initialize_tree(eqn)
            self.rsstring, self.lsstring = self.tree_to_string(self.rstree), self.tree_to_string(self.lstree)

    def initialize_tree(self, eqn):
        '''takes an equation, splits into two sides. Returns nothing.
        creates self.rstree and self.lsstree from the equation.'''
        index = eqn.find('=')
        left_side, right_side = eqn[:index], eqn[index+1:]
        self.side_w_variable = 'left' if self.variable in left_side else 'right'
        self.rstree, self.lstree = self.tree_base_case_check(right_side), self.tree_base_case_check(left_side)

    def tree_base_case_check(self, side):
        '''takes in one side of the equation. if there's still an
        operation present, keep processsing and return the processed (tree'd)
        side. If not, return itself.'''
        side_string = str(side)
        # check if it is a placeholder, in which case build the tree from the held portion of the equation.
        if side_string in placeholder_list:
            return self.build_tree(self.place_dict[side_string])
        elif any((digit in operator_list or digit in placeholder_list)for digit in side_string[1:]):
            return self.build_tree(side)
        return side

    def build_tree(self, side):
        '''take in a side of an equation, create a tree with the
        operations as nodes, returns that tree.'''
        if all(digit in integer_list or digit in operator_list for digit in side):
            return self.solve_simple(side)

        # If there are parenthesis, go through them first
        if '(' in side:
            self.place_dict = dict()
            self.counter = 0
            self.tree = self.tree_base_case_check(self.par_parse(side))
            return self.tree

        # check first for addition and subtraction
        elif ('-' in side and side.rfind('-') != 0) or '+' in side:
            indexplus, indexmin = side.rfind('+'), side.rfind('-')
            if indexmin != -1 and side[indexmin-1] in operator_list:
                indexmin = side[:indexmin].rfind('-')
            if indexmin > indexplus:
                index, element = indexmin, '-'
            elif indexplus > indexmin:
                index, element = indexplus, '+'
            elif indexplus == -1 and indexmin == -1:
                element = 'NO'
            if element == '+' or element == '-':
                left_ele, right_ele = side[:index], side[index+1:]
                self.tree = (element, self.tree_base_case_check(left_ele), self.tree_base_case_check(right_ele))
                return self.tree

        # then check for division or multiplication
        elif '/' in side or '*' in side:
            indexdiv, indexmul = side.rfind('/'), side.rfind('*')
            if indexmul > indexdiv:
                index, element = indexmul, '*'
            else:
                index, element = indexdiv, '/'
            left_ele, right_ele = side[:index], side[index+1:]
            self.tree = (element, self.tree_base_case_check(left_ele), self.tree_base_case_check(right_ele))
            return self.tree

        # then check for powers
        elif '^' in side:
            indexcarrot = side.rfind('^')
            left_ele, right_ele = side[:indexcarrot], side[indexcarrot+1:]
            self.tree = ('^', self.tree_base_case_check(left_ele), self.tree_base_case_check(right_ele))
            return self.tree

    def par_parse(self, equation):
        '''takes an equation and returns a parsed version of it.
        also adds placeholders and what they represent into self.place_dict'''
        while '(' in str(equation):
            start_par = equation.find('(')

            ind_open = equation[start_par + 1:].find('(')
            ind_close = equation[start_par + 1:].find(')')

            # if it closes before a new one opens, add it to self.place_dict
            if ind_close < ind_open or ind_open == -1:
                p_holder = placeholder_list[self.counter]
                inside_par = equation[start_par + 1:ind_close + start_par + 1]
                if all(digit in operator_list or digit in integer_list for digit in inside_par):
                    p_holder = self.solve_simple(inside_par)
                else:
                    self.place_dict[p_holder] = inside_par
                    self.counter += 1
                equation = equation[:start_par] + p_holder + equation[ind_close + start_par + 2:]

            # if another one opens, recursively go inside until it gets to one that closes
            elif ind_open < ind_close:
                last_close = self.find_corr_close_par(equation[start_par+1:]) + start_par
                outer_nest = equation[ind_open + start_par + 1:last_close+1]
                parsed_outer = self.par_parse(outer_nest)
                equation = equation[:ind_open + start_par + 1] + parsed_outer + equation[last_close+1:]
        return equation

    def find_corr_close_par(self, equation):
        '''given an equation starting after a beginning (, returns the index of the corresponding )'''
        counter = 1
        index = 0
        for digit in equation:
            if digit != '(' and digit != ')':
                index += 1
            else:
                if digit == '(':
                    counter += 1
                elif digit == ')':
                    counter += -1
                if counter == 0:
                    return index
                index += 1

    def tree_to_string(self, tree):
        '''takes a tree. returns a string the tree represents.'''
        equation_string = ''
        if type(tree) == tuple:
            if type(tree[1]) == str and type(tree[2]) == tuple:
                equation_string = tree[1] + tree[0] + self.tree_to_string(tree[2])
            elif type(tree[1]) == tuple and type(tree[2]) == str:
                equation_string = self.tree_to_string(tree[1]) + tree[0] + tree[2]
            elif type(tree[1]) == str and type(tree[2]) == str:
                equation_string = '(' + tree[1] + tree[0] + tree[2] + ')'
            elif type(tree[1]) == tuple and type(tree[2]) == tuple:
                equation_string = self.tree_to_string(tree[1]) + tree[0] + self.tree_to_string(tree[2])
        else:
            equation_string = tree
        return equation_string

    def solve_algebra(self):
        '''takes the self.trees and self.strings and does the next appropriate
        operation to them'''
        # vt - variable side tree; vs - variable side string; nvt - nonvariable side tree; nvs - nonvariable side string
        if self.side_w_variable == 'left':
            vt_vs_nvt_nvs = [self.lstree, self.lsstring, self.rstree, self.rsstring]
        else:
            vt_vs_nvt_nvs = [self.rstree, self.rsstring, self.lstree, self.lsstring]

        # figures out where in the tree the variable is
        if self.variable in str(vt_vs_nvt_nvs[0][2]):
            mov_idx, keep_idx = 1, 2
        elif self.variable in str(vt_vs_nvt_nvs[0][1]):
            mov_idx, keep_idx = 2, 1

        # input the right variables and signs and do the operation
        if vt_vs_nvt_nvs[0][0] == '+':
            vt_vs_nvt_nvs[0], vt_vs_nvt_nvs[3] = self.do_op(vt_vs_nvt_nvs[0], vt_vs_nvt_nvs[3], '-', mov_idx, keep_idx)
        elif vt_vs_nvt_nvs[0][0] == '-':
            vt_vs_nvt_nvs[0], vt_vs_nvt_nvs[3] = self.do_op(vt_vs_nvt_nvs[0], vt_vs_nvt_nvs[3], '+')
        elif vt_vs_nvt_nvs[0][0] == '*':
            vt_vs_nvt_nvs[0], vt_vs_nvt_nvs[3] = self.do_op(vt_vs_nvt_nvs[0], vt_vs_nvt_nvs[3], '/', mov_idx, keep_idx)
        elif vt_vs_nvt_nvs[0][0] == '/':
            vt_vs_nvt_nvs[0], vt_vs_nvt_nvs[3] = self.do_op(vt_vs_nvt_nvs[0], vt_vs_nvt_nvs[3], '*')
        elif vt_vs_nvt_nvs[0][0] == '^':
            vt_vs_nvt_nvs[0], vt_vs_nvt_nvs[3] = self.logs_what(vt_vs_nvt_nvs[0], vt_vs_nvt_nvs[3], mov_idx, keep_idx)

        # reassign to the attributes
        vt_vs_nvt_nvs[1] = self.tree_to_string(vt_vs_nvt_nvs[0])
        vt_vs_nvt_nvs[2] = self.tree_base_case_check(vt_vs_nvt_nvs[3])
        if self.side_w_variable == 'left':
            self.lstree, self.lsstring, self.rstree, self.rsstring = vt_vs_nvt_nvs
        elif self.side_w_variable == 'right':
            self.rstree, self.rsstring, self.lstree, self.lsstring = vt_vs_nvt_nvs
        self.side_w_variable = 'left' if self.variable in str(self.lsstring) else 'right'

    def do_op(self, var_side_tree, non_var_str, string, mov_idx=2, keep_idx=1):
        '''takes the tree of the side with the variable, the strong of the opposite side,
        and a string of the operation it needs to do, snips the operation
        from the tree and moves it to the other side string. returns new tree and string.'''
        non_var_str = str(non_var_str) + string + self.tree_to_string(var_side_tree[mov_idx])
        non_var_str = eval(non_var_str) if self.variable not in non_var_str else non_var_str
        var_side_tree = var_side_tree[keep_idx]
        return var_side_tree, non_var_str

    def logs_what(self, var_side_tree, non_var_str, mov_idx, keep_idx):
        '''is the do_op operation but with powers.'''
        if keep_idx == 1:
            non_var_str = str(eval(str(non_var_str) + '**' + str(eval(('1.0' + '/' + str(var_side_tree[mov_idx]))))))
            var_side_tree = var_side_tree[keep_idx]
            return var_side_tree, non_var_str
        if keep_idx == 2:
            non_var_str = m.log10(eval(str(non_var_str)))/m.log10(eval(str(var_side_tree[mov_idx])))
            var_side_tree = var_side_tree[keep_idx]
            return var_side_tree, non_var_str

    def parse_var_mul(self, raw_eqn):
        '''if there is a variable with a coefficient, put a multiplication
        sign between them'''
        for digit in raw_eqn:
            if digit == self.variable or digit == '(':
                index = raw_eqn.find(digit) if digit == self.variable else raw_eqn.find(digit)
                if (raw_eqn[index-1] in integer_list or raw_eqn[index-1] in variable_list) and index !=0:
                    raw_eqn = raw_eqn[0:index] + '*' + raw_eqn[index:]
                    raw_eqn = self.parse_var_mul(raw_eqn)
        return raw_eqn

    def determine_problem(self):
        '''figures out what type of problem it is'''
        if any(digit in variable_list for digit in self.eqn):
            try:
                self.initialize_algebra(self.eqn)
                while self.lstree != self.lsstring or self.rstree != self.rsstring:
                    self.solve_algebra()
                if self.rsstring == self.variable:
                    return self.lsstring
                elif self.lsstring == self.variable:
                    return self.rsstring
            except ValueError as err:
                print(err)
                self.pub_behave.publish('sad')
                self.check_completion()
                self.pub_speak.publish("I can't solve that. Give me another problem.")
                time.sleep(2)
                self.num_demos += 1
        elif all(digit in variable_list or digit in integer_list or digit in operator_list for digit in self.eqn):
            actual_ops = operator_list[:4]
            for n in range(len(self.eqn)-1):
                print(self.eqn[n])
                if self.eqn[n] in actual_ops and self.eqn[n+1] in actual_ops and self.eqn[n+1] != '-':
                    self.pub_behave.publish('sad')
                    self.check_completion()
                    self.pub_speak.publish("That's not how to write equations. Give me another problem.")
                    self.num_demos += 1
            return self.solve_simple(self.eqn)
        else:
            self.pub_behave.publish('sad')
            self.check_completion()
            self.pub_speak.publish("I can't solve that. Give me another problem.")
            time.sleep(2)
            self.num_demos += 1

    def run(self):
        '''only prints the answer if it's getting a nontrivial input
        will only print the output once '''
        self.num_demos = 0
        answer = ''
        print "running"
        while self.num_demos < 3:
            self.pub_behave.publish("look")
            self.check_completion()
            print "waiting for a problem"
            time.sleep(3)

            while self.eqn == '':
                pass
            print "The eqn I found is ", self.eqn
            answer = self.determine_problem()
            if answer != '' or answer != 'None':
                print "ANSWER: ", answer
                msg = Edwin_Shape()
                msg.shape = str(answer)
                msg.x = -500
                msg.y = 5700
                msg.z = -810
                self.pub.publish(msg)
                self.num_demos += 1
                print "waiting on writing"
                while self.writing_status == 0:
                    pass
                print "problem done"
                self.eqn = ''
                self.writing_status = 0
        self.pub_speak.publish("There. Your homework is done.")
        time.sleep(3)


if __name__ == '__main__':
    ctr = Calculator()
    ctr.run()

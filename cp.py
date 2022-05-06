from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from ortools.sat.python import cp_model
import time
import collections

import sys
import numpy as np




class VarArrayAndObjectiveSolutionPrinter(cp_model.CpSolverSolutionCallback):
    """Print intermediate solutions."""

    def __init__(self, variables):
        cp_model.CpSolverSolutionCallback.__init__(self)
        self.__variables = variables
        self.__solution_count = 0
        self.start = time.time()
        self.start_interval = time.time()

    def on_solution_callback(self):
        t1 = time.time()
        time_used = t1 - self.start
        interval_used = t1 - self.start_interval
        self.start_interval = t1
        print('Interval using %.4f, Accu using %.4f, Solution %i' % (interval_used, time_used, self.__solution_count), end = ', ')
        print('objective value = %i' % self.ObjectiveValue())
        for v in self.__variables:
            print('  %s = %i' % (v, self.Value(v)), end=',')
        print()
        self.__solution_count += 1

    def solution_count(self):
        return self.__solution_count



def cp(edges, node_count, max_space = -1, max_minutes=10):
    if max_space <= 0: max_space = node_count


    max_space = 95

    adj = collections.defaultdict(set)  
    for edge in edges:
        adj[edge[0]].add(edge[1])
        adj[edge[1]].add(edge[0])

    model = cp_model.CpModel()
    colors = []

    for i in range(node_count):
        colors.append(
            model.NewIntVar(0, max_space-1, "color_{}".format(i))
        )

    print('Number of variables:', len(model._CpModel__model.variables))

    
    for node in range(node_count):
        for neigh in adj[node]:
            if node != neigh:
                model.Add(colors[node] != colors[neigh])

    model.Minimize(0)

    print('Number of constraints: ', len(model._CpModel__model.constraints))


    solver = cp_model.CpSolver()
    solver.parameters.max_time_in_seconds = 60*max_minutes
    solution_printer = VarArrayAndObjectiveSolutionPrinter(colors)
    status = solver.SolveWithSolutionCallback(model, solution_printer)
    print('----------------')
    print('Status       : %s' % solver.StatusName(status))
    print('#sol found   : %i' % solution_printer.solution_count())
    print('Branches     : %i' % solver.NumBranches())
    print('Wall time    : %f s' % solver.WallTime())

    obj = solver.ObjectiveValue()
    solution = [0]*len(colors)
    for idx, color in enumerate(colors):
        solution[idx] = solver.Value(color)
    is_optimal = -1
    if status == cp_model.OPTIMAL:
        is_optimal = 1
    elif status == cp_model.FEASIBLE:
        is_optimal = 0
    print('Obj          : %s' %(obj))
    print('Solution     : %s' %(','.join(map(str, solution))))
    print('----------------')

    objective = len(collections.Counter(solution))
    return objective, solution

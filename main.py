from simpleai.search.models import SearchProblem
from simpleai.search.traditional import astar, depth_first, greedy, uniform_cost, breadth_first, limited_depth_first, \
    iterative_limited_depth_first
from simpleai.search.viewers import BaseViewer
from time import perf_counter

class NQueensProblem(SearchProblem):

    def __init__(self, N):

        super(NQueensProblem, self).__init__(N)
        self._actions = []

        for i in range(len(N)):
            for j in range(len(N)):
                self._actions.append(("Move Queen C: " + str(i) + " R: " + str(j), i, j))
        print(self._actions)

    def actions(self, state):
        return self._actions

    def is_goal(self, state):
        return int(self.count_attacking_pairs(state)) == 0

    def result(self, state, action):
        return state[:action[1]][::-1] + state[action[1]:]

    # just PA3
    def heuristic(self, state):
        return int(self.count_attacking_pairs(state))

    def count_attacking_pairs(self, state):
        sameRaw = 0
        sameDiagonal = 0

        for i in range(len(state)):
            j = i + 1
            for j in range(i + 1, len(state)):
                if state[i] == state[j]:
                    sameRaw += 1
                    continue
                if abs(int(i) - int(j)) == abs(int(state[j]) - int(state[i])):
                    sameDiagonal += 1
        return str(sameRaw + sameDiagonal)


def set_state(N):
    condition = True
    state = ""
    while condition:


        state = str(input("Please enter a state: "))


        if _is_valid(state) == True:
            condition = False

        else:
            print("invalid state! try again")
            set_state(N)

            condition = False

    return state


def _is_valid(state_str):
    isValid = True
    for c in state_str:
        isValid = c.isdigit() and int(c) <= len(state_str)
        if not isValid:
            break
    return "0" not in state_str and isValid


def main():
    state = set_state(6)
    myviewer = BaseViewer()


    resulting_state = 0
    problem = NQueensProblem(N=state)
    result = ""

    search = ['BFS', 'UCS', 'DFS', 'DLS', 'IDS', 'GREEDY', 'A*']
    for i in search:
        graph_search = True

        if i == 'BFS':
            start = perf_counter()
            result = breadth_first(problem, viewer=myviewer, graph_search=True)
            end = perf_counter()
        elif i == 'UCS':
            start = perf_counter()
            result = uniform_cost(problem, viewer=myviewer, graph_search=True)
            end = perf_counter()
        elif i == 'DFS':
            start = perf_counter()
            result = depth_first(problem, viewer=myviewer, graph_search=True)
            end = perf_counter()
        elif i == 'DLS':
            start = perf_counter()
            result = limited_depth_first(problem, depth_limit=int(len(state) / 2), viewer=myviewer, graph_search=True)
            end = perf_counter()
        elif i == 'IDS':
            start = perf_counter()
            result = iterative_limited_depth_first(problem, viewer=myviewer, graph_search=True)
            end = perf_counter()
        elif i == 'GREEDY':
            start = perf_counter()
            result = greedy(problem, viewer=myviewer, graph_search=True)
            end = perf_counter()
        else:
            start = perf_counter()
            result = astar(problem, viewer=myviewer, graph_search=True)
            end = perf_counter()
        print("***************************\n")

        print("Graph_search? " + str(graph_search))
        print(i)
        print("Resulting path: ")
        print(result.path())
        print("Resulting state: " + str(result.path()[-1][-1]))
        print("Total cost: " + str(result.cost))
        print("Viewer Stats: ")
        print(myviewer.stats)
        print("Time taken: {0:.8f} seconds".format(end-start))
        print("Correct solution? : ", NQueensProblem(N=state).count_attacking_pairs(result.path()[-1][-1]) == "0")

        print("***************************\n")


if __name__ == '__main__':
    main()

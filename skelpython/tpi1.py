#STUDENT NAME: Rodrigo Abreu
#STUDENT NUMBER: 113626

#DISCUSSED TPI-1 WITH: (names and numbers):
#Discutimos o exercício 7.
#Eduardo Lopes, 103070
#Ricardo Antunes, 115243

#Este link foi útil para algumas alineas.
#https://stackoverflow.com/questions/54300715/python-3-list-sorting-with-a-tie-breaker

from tree_search import *
from strips import *
from blocksworld import *

class MyNode(SearchNode):

    def __init__(self,state,parent,depth=None,cost=None,heuristic=None,action=None):
        super().__init__(state,parent)
        #ADD HERE ANY CODE YOU NEED
        self.depth = self.get_depth()
        self.cost = cost
        self.heuristic = heuristic
        self.action = action

    # função para calcular o depth do respetivo nó
    def get_depth(self):
       return 0 if self.parent == None else self.parent.get_depth() + 1

class MyTree(SearchTree):

    def __init__(self,problem, strategy='breadth',improve=False):
        super().__init__(problem,strategy)
        #ADD HERE ANY CODE YOU NEED
        root = MyNode(problem.initial, None, cost=0, heuristic=self.problem.domain.heuristic(problem.initial, problem.goal))
        self.open_nodes = [root]
        self.num_solution = 0
        self.num_skipped = 0
        self.num_closed = 0

        self.improve = improve

    def astar_add_to_open(self,lnewnodes):
        # adicionar a lista de novos open nodes à lista open_nodes
        self.open_nodes.extend(lnewnodes)

        # ordenar a lista open_nodes de acordo com os criterios do enunciado
        self.open_nodes.sort(key=lambda node: (node.cost + node.heuristic, node.depth, str(node.state)))


    def informeddepth_add_to_open(self,lnewnodes):
        # IMPLEMENTAR AQUI
        # primeiro implementamos o a* para ordenar os novos open nodes
        lnewnodes.sort(key=lambda node: (node.cost + node.heuristic, str(node.state)))

        # depois adiciona-se ao início da lista por ser depth-first search
        self.open_nodes[:0] = lnewnodes
        

    def search2(self):
        # IMPLEMENTAR AQUI
        while self.open_nodes != []:
            node = self.open_nodes.pop(0)
            if self.problem.goal_test(node.state):
                self.num_solution += 1

                if self.solution == None or self.solution.cost>node.cost:
                    self.solution = node
                
        # se o improve não estiver ligado retorna-se o primeiro resultado que seja solução
                if not self.improve:
                    return self.get_path(self.solution)

        # caso o nó tenha maior custo em relçaão à sua solução atual, ele é skipped
        # incrementa-se o atributos num_skipped 
            elif self.solution != None and (node.cost + node.heuristic) >= self.solution.cost:
                self.num_skipped+=1
                continue

            else:
                # o nó vai ser expandido aqui, logo considera-se fechado
                # incrementa-se o atributo num_closed
                self.num_closed += 1
                lnewnodes = []

                for a in self.problem.domain.actions(node.state):
                    newstate = self.problem.domain.result(node.state,a)
                    if newstate not in self.get_path(node):
                        action_cost = self.problem.domain.cost(node.state, a)
                        node_heuristic = self.problem.domain.heuristic(newstate, self.problem.goal)

                        newnode = MyNode(newstate, node,  cost=(action_cost + node.cost), action=a, heuristic=node_heuristic)
                        lnewnodes.append(newnode)
                self.add_to_open(lnewnodes)
        
        return self.get_path(self.solution)
 
    def check_admissible(self,node):
        #Assume that the given "node" is a solution node
        #IMPLEMENT HERE

        # guardar o custo da solução
        solution_cost = node.cost

        # iterar até à root
        while node is not None:
            cost = solution_cost - node.cost
            # verificar a admissibilidade, se a heuristica é maior que o custo até à solução
            if node.heuristic > cost:
                return False
            
            node = node.parent
        return True


    def get_plan(self,node):
        # IMPLEMENT HERE

        # para quando o parent for None (root), (a ação da root para um nó é None)
        if node.parent is None:
            return []
        
        # incrementar a lista de ações
        return self.get_plan(node.parent) + [node.action]

    # if needed, auxiliary methods can be added here

    # o numero de open nodes é sempre a quantidade de nós na lista open_nodes 
    @property
    def num_open(self):
        return len(self.open_nodes)
    


class MyBlocksWorld(STRIPS):

    def heuristic(self, state, goal):
        #IMPLEMENT HERE
        
        moves = 0
        state_pile = self.organize_piles(state)
        goal_pile = self.organize_piles(goal)

        moves = max(self.h1(state_pile, goal_pile), self.h2(state_pile, goal_pile))

        return moves
    
    def organize_piles(self, state):
        predicates = list(state)

        piles = []
        for pred in state:
            if isinstance(pred, Floor) :
                letter = pred.args[0]
                piles.append([letter])
                
        for pile in piles:
            top_block = pile[-1]  
            while True:
                next_block = self.look_for_ons(top_block, predicates)
                if next_block:
                    pile.append(next_block)
                    top_block = next_block 
                else:
                    break
        return piles
            
    def look_for_ons(self, letter, state):
        for pred in state:
            if isinstance(pred, On) and pred.args[1]==letter:
                letter2 = pred.args[0]
                state.remove(pred)
                return letter2


    def h1(self, state_pile, goal_pile):
        moves = 0

        for goal_stack in goal_pile:
            
            current_stack = next((stack for stack in state_pile if stack and stack[0] == goal_stack[0]), [])

            for i, goal_block in enumerate(goal_stack):
                
                if i < len(current_stack) and current_stack[i] == goal_block:
                    continue
                else:
                    moves += (len(goal_stack) - i)*2 - 1 
                    break 
        return moves
    
    def h2(self, state_pile, goal_pile):
        moves = 0

        for state_stack in state_pile:
            
            goal_stack = next((stack for stack in goal_pile if stack and stack[0] == state_stack[0]), [])
            
            if state_stack in goal_stack:
                moves+=(len(state_stack)-len(goal_stack))*2-1
            
        return moves

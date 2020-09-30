from deap import base, creator, tools, algorithms
import random
from ApfPlanner import *
import numpy
import time
import multiprocessing


VARIANT = 94

start = (1, 10)
goal = (30, 20)
range_x = (0, 35)
range_y = (0, 35)

POPSIZE = 40
GENSIZE = 25

creator.create("FitnessMulti", base.Fitness, weights=(-1.0, -1.0))
creator.create("Individual", list, fitness=creator.FitnessMulti)
toolbox = base.Toolbox()
toolbox.register("attr_KP", random.uniform, 0.1, 7)
toolbox.register("attr_ETA", random.uniform, 10, 500)
toolbox.register("attr_KB", random.uniform, 0.01, 0.9)
toolbox.register("attr_KC", random.uniform, 0.01, 3.14)
toolbox.register("attr_ESCAPE_SIGN", random.uniform, -1, 1)
toolbox.register("individual", tools.initCycle, creator.Individual, (toolbox.attr_KP, toolbox.attr_ETA, toolbox.attr_KB, toolbox.attr_KC, toolbox.attr_ESCAPE_SIGN), n=1)
toolbox.register('population', tools.initRepeat, list, toolbox.individual)

obstacle_manager = ObstacleManager()
if VARIANT == 91:
    obstacle_manager.add_circle('rob1', 8, 7-2-2, 3, 5)
    obstacle_manager.add_circle('rob2', 22, 30, 4, 2)
elif VARIANT == 1:
    pass
elif VARIANT == 2:
    obstacle_manager.add_circle('rob1', 15, 18, 3, 4)
elif VARIANT == 3:
    #obstacle_manager.add_circle('rob1', 15.5, 15, 3, 4)
    obstacle_manager.add_circle('rob1', 15, 18, 3, 4)
elif VARIANT == 4:
    obstacle_manager.add_circle('rob1', 15.5, 15, 3, 4)
elif VARIANT == 5:
    obstacle_manager.add_rectangle("rob1", 15, 12, 8, 8, 45, 3)
    obstacle_manager.add_circle('rob2', 25, 20, 3, 4)
    obstacle_manager.add_circle('rob3', 19, 21, 1, 4)
elif VARIANT == 6:
    obstacle_manager.add_circle('rob1', 34, 20, 3, 4)
elif VARIANT == 7:
    start = (1, 10)
    goal = (30, 10)
    obstacle_manager.add_rectangle('rob1', 14, 10, 3, 7)
elif VARIANT == 92:
    obstacle_manager.add_circle('rob1', 10, 17, 2, 2+2)
    obstacle_manager.add_circle('rob2', 14, 17, 2, 2+2)
    obstacle_manager.add_circle('rob3', 18, 17, 2, 2+2)
    obstacle_manager.add_circle('rob4', 18, 13, 2, 2+2)
    obstacle_manager.add_circle('rob5', 18, 9, 2, 2+2)
    obstacle_manager.add_circle('rob6', 14, 9, 2, 2+2)
    obstacle_manager.add_circle('rob7', 10, 9, 2, 2+2)
    #obstacle_manager.add_rectangle('rob8', 12, 25, 4, 4)
elif VARIANT == 93:
    obstacle_manager.add_circle('rob1', 8, 7 - 2 - 2, 3, 5)
    obstacle_manager.add_circle('rob2', 22, 30, 4, 2)
    obstacle_manager.add_rectangle('rob3', 20, 20, 4, 8, 60, 2 + 1)
    obstacle_manager.add_rectangle('rob4', 10, 13, 9, 4, 90, 4.5)
elif VARIANT == 94:
    obstacle_manager.add_circle('rob1', 10, 17, 2, 2 + 2)
    obstacle_manager.add_circle('rob2', 14, 17, 2, 2 + 2)
    obstacle_manager.add_circle('rob3', 18, 17, 2, 2 + 2)
    obstacle_manager.add_circle('rob4', 18, 13, 2, 2 + 2)
    obstacle_manager.add_circle('rob5', 18, 9, 2, 2 + 2)
    obstacle_manager.add_circle('rob6', 14, 9, 2, 2 + 2)
    obstacle_manager.add_circle('rob7', 10, 9, 2, 2 + 2)
    obstacle_manager.add_rectangle('rob8', 5, 25, 10, 7)

obstacle_manager.add_rectangle('dole', 35/2, 0, 35, 0.1, 0.5)
obstacle_manager.add_rectangle('gore', 35/2, 35, 35, 0.1, 0.5)
obstacle_manager.add_rectangle('levo', 0, 35/2, 35, 0.1, 90, 0.5)
obstacle_manager.add_rectangle('desno', 35, 35/2, 35, 0.1, 90, 0.5)


obstacles_circ, obstacles_rect = obstacle_manager.get_obstacles()


def evaluate(individual):
    KP = individual[0]
    ETA = individual[1]
    KB = individual[2]
    KC = individual[3]
    ESCAPE_SIGN = individual[4]
    STEP_SIZE = 0.4

    planner = ApfPlanner(start, goal, obstacles_circ, obstacles_rect, STEP_SIZE, KP, ETA, KB, KC, ESCAPE_SIGN)
    path, total_dis, total_cnt = planner.run_planner()

    return total_dis, #planner.obs_dist_penalty


# decorator to keep kp positive

def checkBounds():
    def decorator(func):
        def wrapper(*args, **kargs):
            offspring = func(*args, **kargs)
            for child in offspring:
                if child[0] < 0:
                    child[0] = 0.1
            return offspring
        return wrapper
    return decorator

toolbox.register('evaluate', evaluate)
toolbox.register('mate', tools.cxOnePoint)
toolbox.register("mutate", tools.mutGaussian, mu=0.0, sigma=0.2, indpb=0.2)
toolbox.register("select", tools.selTournament, tournsize=3)

toolbox.decorate("mate", checkBounds())
toolbox.decorate("mutate", checkBounds())

if __name__ == "__main__":
    startTime = time.time()

    pool = multiprocessing.Pool()
    toolbox.register("map", pool.map)

    pop = toolbox.population(n=POPSIZE)
    print(pop)
    hof = tools.HallOfFame(1)
    stats = tools.Statistics(lambda ind: ind.fitness.values)
    #stats.register("avg", numpy.mean)
    stats.register("min", numpy.min, axis=0)
    #stats.register('min_obs_dist', numpy.max, axis=1)
    #stats.register("max", numpy.max)

    pop, logbook = algorithms.eaSimple(pop, toolbox, cxpb=0.35, mutpb=0.55, ngen=GENSIZE, stats=stats, halloffame=hof, verbose=True)

    stopTime = time.time()
    print('solved in: ' + str(stopTime - startTime) + ' seconds')

    print(hof[0])
    print('[{:.2f}, {:.2f}, {:.2f}, {:.2f}, {:.2f}]'.format(*hof[0]))

    # # # # # # # # # # #
    # plot the result
    STEP_SIZE = 0.4
    KP, ETA, KB, KC, ESCAPE_SIGN = hof[0]
    figure, axes = plt.subplots()
    plt.axis([range_x[0], range_x[1], range_y[0], range_y[1]])
    plt.plot(*goal, 'rx')

    obstacles_circ, obstacles_rect = obstacle_manager.get_obstacles()

    planner = ApfPlanner(start, goal, obstacles_circ, obstacles_rect, STEP_SIZE, KP, ETA, KB, KC, ESCAPE_SIGN)

    planner.plot_obstacles(obstacles_circ, obstacles_rect, axes)

    startTime = time.time()

    path, total_dis, total_cnt = planner.run_planner()

    stopTime = time.time()
    print('solved in: ' + str(stopTime - startTime) + ' seconds')

    seg_path = planner.get_path_every_nth_point(path)
    planner.plot_path(seg_path)

    print('Distance = ' + str(total_dis))

    plt.show()

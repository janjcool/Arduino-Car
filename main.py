import matplotlib.pyplot as plt
import numpy as np
from matplotlib.ticker import MultipleLocator

TIME_AXIS_RANGE: int = 150  # in seconds
POSITION_AXIS_RANGE: int = 320  # in cm

# INIT VARIABLES
LIGHT_POINTS = [60, 160, 260]
x_green_list = []
x_red_list = []
x_yellow_list = []


def calculate_routes(x_offset, y_offset, color, go_trough_yellow, light1=True, light2=True, light3=True,
                     disable_slow_routes=False, slow_threshold=60):
    # ------------ CALCULATE ROUTES ------------ #
    ricos = np.arange(0.05, 20, 0.005)
    good_ricos = []

    for rico in ricos:
        good_rico = True
        light_pos_list = []

        if light1 is True:
            light_pos_list.append(LIGHT_POINTS[0])
        if light2 is True:
            light_pos_list.append(LIGHT_POINTS[1])
        if light3 is True:
            light_pos_list.append(LIGHT_POINTS[2])

        for light_pos in light_pos_list:

            if go_trough_yellow is False:
                for x_temp_list in x_yellow_list:
                    begin, end = x_temp_list[0], x_temp_list[1]
                    x = light_pos / rico + x_offset + y_offset

                    if begin < x < end:
                        good_rico = False

            for x_temp_list in x_red_list:
                begin, end = x_temp_list[0], x_temp_list[6]
                x = light_pos / rico + x_offset + y_offset

                if begin < x < end:
                    good_rico = False

        if good_rico is True:
            good_ricos.append(rico)

    # ------------ DISABLE TO SLOW ROUTES ------------ #
    if disable_slow_routes is True:
        print("All routes slower than " + str(slow_threshold) + " will be disabled")
        new_ricos = []

        for rico in good_ricos:
            x = slow_threshold / rico + x_offset + y_offset
            if x <= 120:
                new_ricos.append(rico)

        good_ricos = new_ricos

    # ------------ DISPLAY POSSIBLE ROUTES ------------ #
    print("The good ricos are: " + str(good_ricos))

    for rico in good_ricos:
        plt.axline((0 + x_offset, y_offset), (1 + x_offset, rico + y_offset), color=color)


def main():
    fig, ax = plt.subplots()
    ax.yaxis.set_minor_locator(MultipleLocator(2.5))
    ax.xaxis.set_minor_locator(MultipleLocator(2))
    ax.xaxis.set_major_locator(MultipleLocator(10))
    ax.yaxis.set_major_locator(MultipleLocator(25))
    plt.grid(color='b', linestyle='-', linewidth=0.1)
    plt.grid(b=True, which='minor', color='b', linewidth=0.05, linestyle='--')
    plt.xlabel('t(s)')
    plt.ylabel('s(cm)')

    x_list = np.arange(0, TIME_AXIS_RANGE + 100, 0.5)

    x_green_list_temp = []
    x_yellow_list_temp = []
    x_red_list_temp = []

    index = -1

    for x in x_list:
        if index <= 5:
            x_green_list_temp.append(x)
        elif index == 6:
            x_yellow_list_temp.append(x - 0.5)
            x_yellow_list_temp.append(x)
            x_red_list_temp.append(x)
        elif index <= 12:
            x_red_list_temp.append(x)
        else:
            x_green_list.append(x_green_list_temp)
            x_yellow_list.append(x_yellow_list_temp)
            x_red_list.append(x_red_list_temp)
            index = 0
            x_green_list_temp = []
            x_yellow_list_temp = []
            x_red_list_temp = []
            x_green_list_temp.append(x - 0.5)
            x_green_list_temp.append(x)

        index += 1

    for list_part in x_yellow_list:
        for index, coordinate in enumerate(list_part):
            if index == 0:
                plt.axline((coordinate, 0), (coordinate, 70), linewidth=1, color="b", linestyle=":")

    for list_part in x_red_list:
        for index, coordinate in enumerate(list_part):
            if index == 6:
                plt.axline((coordinate, 0), (coordinate, 70), linewidth=1, color="b", linestyle=":")

    for y in LIGHT_POINTS:
        for x_temp_list in x_green_list:
            y_temp_list = [y] * len(x_temp_list)
            plt.plot(x_temp_list, y_temp_list, "g")

        for x_temp_list in x_yellow_list:
            y_temp_list = [y] * len(x_temp_list)
            plt.plot(x_temp_list, y_temp_list, "y")

        for x_temp_list in x_red_list:
            y_temp_list = [y] * len(x_temp_list)
            plt.plot(x_temp_list, y_temp_list, "r")

    # ------------ MAKE ROUTES ------------ #

    calculate_routes(0, 0, "green", True, True, True, True)

    plt.axline((0, 0), (1, 35), color="red")
    plt.axline((0, 0), (1, 8), color="purple")
    plt.axline((0, 0), (1, 7.225), color="purple")

    plt.axis([0, TIME_AXIS_RANGE, 0, POSITION_AXIS_RANGE])
    plt.show()


if __name__ == '__main__':
    main()

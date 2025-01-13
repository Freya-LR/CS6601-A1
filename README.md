# Spring 2025 - CS6601 - Assignment 1

## Instructor: Dr. Thad Starner

## Deadline: Monday, January 27th, 7:59 am EST

#### Released: Monday, January 13th, 8:00 am EST

Search is an integral part of AI. It helps in problem solving across a wide variety of domains where a solution isn’t immediately clear. Your task is to implement several search algorithms that will calculate a route between two points in Romania while seeking to minimize time and space cost. We will be using an undirected network representing a map of Romania (and an optional Atlanta graph used for the Race!).

### Table of Contents
- [Setup](#setup)
- [Dependencies](#dependencies)
- [Jupyter](#jupyter)
- [Jupyter Tips](#jupyter-tips)
- [Submission](#submission)
- [Custom Tests](#custom-tests)

<a name="setup"/></a>
### Setup

Create a conda environment if you have not already. For example:

```
conda create --name a1_env python=3.9 -y
```

Activate the environment:

```
conda activate a1_env
```

In case you used a different environment name to create the conda environment, you can list all environments you have on your machine by running `conda env list`. You can always refer back to the instructions provided in Assignment 0 for managing conda environments.


<a name="setup"/></a>
### Dependencies

Install the necessary libraries for this assignment after activating your conda environment and navigating to the correct directory.

```
pip install -r requirements.txt
```

<a name="jupyter"/></a>
## Jupyter

To open the Jupyter Notebook, run:

```
jupyter notebook
```

This should automatically open the `notebook.ipynb` as a Jupyter Notebook. If it doesn't automatically open, you can access the Jupyter Notebook at [http://localhost:8888](http://localhost:8888/) in your browser.

<a name="jupyter-tips"/></a>
## Jupyter Tips

Hopefully, [Assignment 0](https://github.gatech.edu/omscs6601/assignment_0/) got you pretty comfortable with Jupyter or at the very least addressed the major things that you may run into during this project. That said, Jupyter can take some getting used to, so here is a compilation of some things to watch out for specifically when it comes to Jupyter in a sort-of FAQs-like style

**1. My Jupyter notebook does not seem to be starting up or my kernel is not starting correctly.**

Ans: This probably has to do with activating virtual environments. If you followed the setup instructions exactly, then you should activate your conda environment using `conda activate <environment_name>` from the Anaconda Prompt and start Jupyter Notebook from there.

**2. I was running cell xxx when I opened up my notebook again and something or the other seems to have broken.**

Ans: This is one thing that is very different between IDEs like PyCharm and Jupyter Notebook. In Jupyter, every time you open a notebook, you should run all the cells that a cell depends on before running that cell. This goes for cells that are out of order too (if cell 5 depends on values set in cell 4 and 6, you need to run 4 and 6 before 5). Using the "Run All" command and its variants (found in the "Cell" dropdown menu above) should help you when you're in a situation like this.

**3. The value of a variable in one of my cells is not what I expected it to be? What could have happened?**

Ans: You may have run a cell that modifies that variable too many times. Look at the "counter" example in assignment 0. First, try running `counter = 0` and then `counter += 1`. This way, when you print counter, you get counter = 1, right? Now try running `counter += 1` again, and now when you try to print the variable, you see a value of 2. This is similar to the issue from Question 2. The order in which you run the cells does affect the entire program, so be careful.

<a name="submission"/></a>
## Submission

You will submit multiple files from the directory `submission` to Gradescope after following the instructions in `notebook.ipynb`.

<a name="custom-tests"/></a>
## Custom Tests

There are a series of custom tests provided if you would like to run them through the terminal instead of Jupyter Notebook. They are:

1. `search_basic_tests.py`: Sample basic tests, visualizes the Romania graph.
2. `search_submission_tests_grid.py`: Visualizes the search as a grid.
3. `search_romania_tests.py`: More comprehensive tests on the Romania graph than `search_basic_tests`.
4. `search_atlanta_tests.py`: Tests for the Atlanta graph.
5. `search_case_visualizer.py`: For visualizing specific failed cases of interest on the Romania graph.

To run a full test, you can do something like:

```
python search_romania_tests.py
```

To run a specific test, you can do something like:

```
python search_romania_tests.py SearchRomaniaTests.test_bfs_romania
```

See the code for each test file for more detailed instructions and descriptions.
import tkinter as tk
from tkinter import ttk

from main import run


def submit(debug, back, arms_path, bins_path, summon_component, task_manager_component):
    run(debug, back, arms_path, bins_path, summon_component, task_manager_component)


def main():
    root = tk.Tk()
    root.title('Trash Picking Configuration')
    root.geometry('600x500')
    root.resizable(False, False)
    frame = ttk.Frame(root)
    options = {'padx': 5, 'pady': 5}
    row = 0

    # Debug logging
    debug_var = tk.StringVar()
    debug_label = ttk.Label(frame, text='Debug logging')
    debug_label.grid(column=0, row=row, sticky='W', **options)
    debug_checkbox = ttk.Checkbutton(frame,
                                     variable=debug_var)
    debug_checkbox.grid(column=1, row=row, sticky='W', **options)
    row += 1

    # Background environment
    env_var = tk.StringVar()
    env_label = ttk.Label(frame, text='Show background environment')
    env_label.grid(column=0, row=row, sticky='W', **options)
    env_checkbox = ttk.Checkbutton(frame,
                                   variable=env_var)
    env_checkbox.grid(column=1, row=row, sticky='W', **options)
    row += 1

    # Arms path
    arms_var = tk.StringVar(value='configs/arms_locations.json')
    arms_label = ttk.Label(frame, text='Arms positions JSON file')
    arms_label.grid(column=0, row=row, sticky='W', **options)
    arms_entry = ttk.Entry(frame, textvariable=arms_var)
    arms_entry.grid(column=1, row=row, ipadx=100, sticky='W', **options)
    row += 1

    # Bins path
    bins_var = tk.StringVar(value='configs/trash_bins_locations.json')
    bins_label = ttk.Label(frame, text='Trash bins positions JSON file')
    bins_label.grid(column=0, row=row, sticky='W', **options)
    bins_entry = ttk.Entry(frame, textvariable=bins_var)
    bins_entry.grid(column=1, row=row, ipadx=100, sticky='W', **options)
    row += 1

    # Summon component
    summon_var = tk.StringVar(value='AdvancedRandomSummonComponent')
    summon_label = ttk.Label(frame, text='Summon component')
    summon_label.grid(column=0, row=row, sticky='W', **options)
    summon1 = ttk.Radiobutton(frame,
                              text='Advanced Random Summon Component',
                              value='AdvancedRandomSummonComponent',
                              variable=summon_var)
    summon1.grid(column=1, row=row, sticky='W', **options)
    row += 1
    summon2 = ttk.Radiobutton(frame,
                              text='Random Summon Component',
                              value='RandomSummonComponent',
                              variable=summon_var)
    summon2.grid(column=1, row=row, sticky='W', **options)
    row += 1
    summon3 = ttk.Radiobutton(frame,
                              text='Deterministic Summon Component (Mustard)',
                              value='DeterministicSummonComponent-mustard',
                              variable=summon_var)
    summon3.grid(column=1, row=row, sticky='W', **options)
    row += 1
    summon4 = ttk.Radiobutton(frame,
                              text='Deterministic Summon Component (Metal Can)',
                              value='DeterministicSummonComponent-metal_can',
                              variable=summon_var)
    summon4.grid(column=1, row=row, sticky='W', **options)
    row += 1
    summon5 = ttk.Radiobutton(frame,
                              text='Deterministic Summon Component (Paper Box)',
                              value='DeterministicSummonComponent-paper_box',
                              variable=summon_var)
    summon5.grid(column=1, row=row, sticky='W', **options)
    row += 1


    # Task manager component
    task_var = tk.StringVar(value='AdvancedParallelTaskManager')
    task_label = ttk.Label(frame, text='Task manager component')
    task_label.grid(column=0, row=row, sticky='W', **options)
    task1 = ttk.Radiobutton(frame,
                            text='Advanced Parallel Task Manager',
                            value='AdvancedParallelTaskManager',
                            variable=task_var)
    task1.grid(column=1, row=row, sticky='W', **options)
    row += 1
    task2 = ttk.Radiobutton(frame,
                            text='Parallel Task Manager',
                            value='ParallelTaskManager',
                            variable=task_var)
    task2.grid(column=1, row=row, sticky='W', **options)
    row += 1
    task3 = ttk.Radiobutton(frame,
                            text='Simple Task Manager',
                            value='SimpleTaskManager',
                            variable=task_var)
    task3.grid(column=1, row=row, sticky='W', **options)
    row += 1

    # Submit button
    def on_submit():
        debug = debug_var.get() == '1'
        back = env_var.get() == '1'
        arms_path = arms_var.get()
        bins_path = bins_var.get()
        summon_component = summon_var.get()
        task_manager_component = task_var.get()
        root.destroy()
        submit(debug, back, arms_path, bins_path, summon_component, task_manager_component)

    submit_button = ttk.Button(frame, text='Submit')
    submit_button.grid(column=1, row=row, sticky='W', **options)
    submit_button.configure(command=on_submit)
    row += 1

    frame.grid(padx=10, pady=10)
    root.mainloop()


if __name__ == '__main__':
    main()

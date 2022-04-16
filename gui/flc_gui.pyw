# -*- coding: utf-8 -*-

from tkinter import *
import flc_com
from flc_menu import FlcMenu
import sys

fl = None  # fitolamp object
com_name = ""  # port name
com_list = flc_com.get_avail_ports()


def app_exit():
    print("Tkinter window closed")
    global fl
    root.destroy()
    if fl: fl.close()


root = Tk()
root.title("Fitolamp")
root.geometry('240x600')
root.protocol('WM_DELETE_WINDOW', app_exit)


def show_start_menu():
    com_scan_btn.pack()
    com_olbl.pack()
    lframe.pack()
    com_conn_btn.pack()
    com_disc_btn.pack_forget()
    com_slbl.config(text="Select port to connect")

    flm.hide()


def show_app_menu():
    com_slbl.config(text="Connected to " + com_name)
    com_olbl.pack_forget()
    com_scan_btn.pack_forget()
    lframe.pack_forget()
    com_conn_btn.pack_forget()
    com_disc_btn.pack()

    flm.show()
    flm.update_status()
    flm.res_txt.delete(0.0, END)
    flm.res_txt.insert(0.0, "Device status: OK\n")


def com_scan_cmd():
    global com_name
    global fl
    for item in com_list:
        try:
            fl = flc_com.Fitolamp(item)
        except flc_com.OpenPortException:
            com_slbl.config(text="Unable to open port '" + item + "'")
        else:
            if fl.get_status() == "OK":
                com_name = item
                flm.set_dev(fl)
                show_app_menu()
                break
            else:
                fl.close()
                com_slbl.config(text="No device found. Try to choose from list")


def com_conn_cmd():
    print(com_name)
    global fl
    try:
        fl = flc_com.Fitolamp(com_name)
    except flc_com.OpenPortException:
        com_slbl.config(text="Unable to open port '" + com_name + "'")
    else:
        if fl.get_status() == "OK":
            flm.set_dev(fl)
            show_app_menu()
        else:
            fl.close()
            com_slbl.config(text="No device found on this port")


def com_disc_cmd():
    global fl
    fl.close()
    flm.set_dev(None)
    show_start_menu()


flm = FlcMenu(root)

com_scan_btn = Button(root, text="Scan for device", command=com_scan_cmd)
com_conn_btn = Button(root, text="Connect", command=com_conn_cmd)
com_disc_btn = Button(root, text="Disconnect", command=com_disc_cmd)
com_slbl = Label(root, text="Select port to connect", fg='black')
com_olbl = Label(root, text="Or choose from available", fg='black')


def select_item(event):
    global com_name
    com_name = (listbox.get(listbox.curselection()))
lframe = Frame(root)
sbar = Scrollbar(lframe)
listbox = Listbox(lframe, width=10, height=5)
sbar.config(command=listbox.yview)
listbox.config(yscrollcommand=sbar.set)
listbox.bind('<<ListboxSelect>>', select_item)

for item in com_list:
    listbox.insert(END, item)

com_slbl.pack()
com_scan_btn.pack()
com_olbl.pack()
lframe.pack()
sbar.pack(side=RIGHT, fill=Y)
listbox.pack(side=LEFT, expand=YES)
com_conn_btn.pack()

try:
    root.mainloop()
except:
    print("Exception: " + sys.exc_info()[:2])

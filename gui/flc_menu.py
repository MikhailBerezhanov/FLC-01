from tkinter import *


class FlcMenu(Frame):
    def __init__(self, parent=None, dev=None, **options):
        Frame.__init__(self, parent, options)

        self.dev = dev
        self.update_time = 2000

        self.state_lfr = LabelFrame(self, text="Power state", width=20)
        self.state_lfr.pack()
        self.power_entr = Entry(self.state_lfr, width=10)
        self.power_entr.pack(padx=28, pady=5)
        
        self.currtime_lfr = LabelFrame(self, text="Current time", width=20)
        self.currtime_lfr.pack()
        self.currtime_btn = Button(self.currtime_lfr, text="set", command=self.set_currtime)
        self.currtime_btn.pack(side=RIGHT, padx=10)
        self.currtime_entr = Entry(self.currtime_lfr, width=10)
        self.currtime_entr.pack(padx=5)

        self.starttime_lfr = LabelFrame(self, text="Start time", width=20)
        self.starttime_lfr.pack()
        self.starttime_btn = Button(self.starttime_lfr, text="set", command=self.set_starttime)
        self.starttime_btn.pack(side=RIGHT, padx=10)
        self.starttime_entr = Entry(self.starttime_lfr, width=10)
        self.starttime_entr.pack(padx=5)

        self.stoptime_lfr = LabelFrame(self, text="Stop time", width=20)
        self.stoptime_lfr.pack()
        self.stoptime_btn = Button(self.stoptime_lfr, text="set", command=self.set_stoptime)
        self.stoptime_btn.pack(side=RIGHT, padx=10)
        self.stoptime_entr = Entry(self.stoptime_lfr, width=10)
        self.stoptime_entr.pack(padx=5)

        self.smooth_lfr = LabelFrame(self, text="Smooth control", width=20)
        self.smooth_lfr.pack()
        self.smooth_var = StringVar()
        self.smooth_var.set("False")
        Button(self.smooth_lfr, text="set", command=self.set_smooth).pack(side=RIGHT, padx=10)
        Checkbutton(self.smooth_lfr, text="Enabled", variable=self.smooth_var,
                    onvalue="True", offvalue="False").pack(padx=2.5)
        
        self.pwr_dict = {}
        self.pwr_lfr = LabelFrame(self, text="Channels power")
        self.pwr_lfr.pack()
        for key in ["CH-1", "CH-2", "CH-3"]:
            tmp = LabelFrame(self.pwr_lfr, text=key)
            tmp.pack(side=LEFT, padx=2, pady=2)
            self.pwr_dict[key] = IntVar()
            self.pwr_dict[key].set(0)
            Scale(tmp, variable=self.pwr_dict[key], from_=100, to=0,
                  tickinterval=50, resolution=1, length=150).pack()
            Button(tmp, text="set", command= (lambda chnl=key: self.set_pwr(chnl))).pack()

        self.res_txt = Text(self, bg="white", fg='black')
        self.res_txt.pack(side=BOTTOM, pady=10)

    def set_dev(self, dev):
        self.dev = dev
        
    def set_currtime(self):
        val = self.currtime_entr.get()
        result = "Set current time: " + self.dev.set_current_time(val) + '\n'
        self.res_txt.insert(0.0, result)
        self.after(self.update_time, self.update_status)
        #self.update_status()
        
    def set_starttime(self):
        val = self.starttime_entr.get()
        result = "Set start time: " + self.dev.set_start_time(val) + '\n'
        self.res_txt.insert(0.0, result)
        self.after(self.update_time, self.update_status)
        #self.update_status()

    def set_stoptime(self):
        val = self.stoptime_entr.get()
        result = "Set stop time: " + self.dev.set_stop_time(val) + '\n'
        self.res_txt.insert(0.0, result)
        self.after(self.update_time, self.update_status)
       # self.update_status()

    def set_pwr(self, chnl):
        val = self.pwr_dict[chnl].get()
        if (chnl == "CH-1"):
            result = "Set CH-1 power: " + self.dev.set_white_power(val) + '\n'
        elif (chnl == "CH-2"):
            result = "Set CH-2 power: " + self.dev.set_red_power(val) + '\n'
        else:
            result = "Set CH-3 power: " + self.dev.set_blue_power(val) + '\n'
        self.res_txt.insert(0.0, result)
        self.update_status()

    def set_smooth(self):
        val = self.smooth_var.get()
        result = "Set smooth control: " + self.dev.set_smooth_control(val) + '\n'
        self.res_txt.insert(0.0, result)
        self.update_status()

    def update_status(self):
        self.dev.get_status()
        
        self.power_entr.delete(0, END)
        self.power_entr.insert(0, str(self.dev.status["power_state"]))

        self.currtime_entr.delete(0, END)
        self.currtime_entr.insert(0, str(self.dev.status["current_dtime"].split(" ")[1]))

        self.starttime_entr.delete(0, END)
        self.starttime_entr.insert(0, str(self.dev.status["start_time"]))

        self.stoptime_entr.delete(0, END)
        self.stoptime_entr.insert(0, str(self.dev.status["stop_time"]))

        self.smooth_var.set(str(self.dev.status["smooth_control"]))
        
        self.pwr_dict["CH-1"].set(self.dev.status["ch1_power"])
        self.pwr_dict["CH-2"].set(self.dev.status["ch2_power"])
        self.pwr_dict["CH-3"].set(self.dev.status["ch3_power"])

        #self.res_txt.delete(0.0, END)

    def show(self):
        self.pack()

    def hide(self):
        self.pack_forget()


if __name__ == "__main__":
    import flc_com
    root = Tk()
    root.geometry('240x600')
    #flc = flc_com.Fitolamp("COM6")
    fl = FlcMenu(root)
    #fl.update_status()
    fl.show()
    #fl.hide()
    root.mainloop()
    print("closing port")
    flc.close()

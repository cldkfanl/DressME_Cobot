from tkinter import Tk, Frame, Label, Scale, Button, messagebox, StringVar, simpledialog
from tkinter.ttk import Combobox  # Combobox를 불러옵니다
import serial
import serial.tools.list_ports

import queue

class RobotArmControl:
    def __init__(self, master):
        self.master = master
        self.master.title("Robot Arm Control")

        # Serial Communication
        self.setup_serial_frame()

        # Robot Arm Frame
        self.setup_robot_arm()

        # Spacer Frame
        self.setup_spacer_frame(pady=20)

        self.action_queue = queue.Queue()

    def setup_serial_frame(self):
        serial_frame = Frame(self.master, width=300)
        serial_frame.pack()

        serial_label = Label(serial_frame, text="Serial Port 를 선택하세요", bg="yellow", fg="black", padx=100)
        serial_label.pack()

        # Get available serial ports
        port_list = [port.device for port in serial.tools.list_ports.comports()]
        self.selected_port = StringVar()
        self.selected_port.set(port_list[0] if port_list else "No ports available")

        # 시리얼 버튼과 콤보 박스를 포함할 프레임 생성
        serial_button_frame = Frame(serial_frame)
        serial_button_frame.pack()

        # Serial Port 선택 콤보 박스 생성 및 배치
        port_combobox = Combobox(serial_button_frame, values=port_list, state="readonly", textvariable=self.selected_port)
        port_combobox.pack(side='left', padx=(0, 5))  # 시리얼 버튼과 콤보 박스 사이에 5만큼의 패딩 추가
        port_combobox.bind("<<ComboboxSelected>>", self.update_selected_port)  # 선택이 변경될 때마다 변수 업데이트

        # Open Serial 버튼 생성 및 배치
        open_serial_button = Button(serial_button_frame, text="Open Serial", height=2, width=15,fg="black", command=self.open_serial)
        open_serial_button.pack(side='left', padx=(0, 5))  # 시리얼 버튼과 콤보 박스 사이에 5만큼의 패딩 추가

        # Close Serial 버튼 생성 및 배치
        close_serial_button = Button(serial_button_frame, text="Close Serial", height=2, width=15, fg="black", command=self.close_serial)
        close_serial_button.pack(side='left')  # 시리얼 버튼과 콤보 박스 사이에 5만큼의 패딩 추가




    def update_selected_port(self, event):
        self.selected_port.set(event.widget.get())  # 콤보박스의 선택값을 selected_port 변수에 업데이트

    def open_serial(self):
        # 사용자가 선택한 포트를 가져옴
        selected_port = self.selected_port.get()

        if selected_port:
            try:
                self.ser = serial.Serial(port=selected_port, baudrate=115200, parity=serial.PARITY_NONE,
                                         stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS, timeout=1)
                messagebox.showinfo("시리얼 연결", f"시리얼 포트 {selected_port}가 열렸습니다")
            except serial.serialutil.SerialException as e:
                messagebox.showerror("시리얼 연결 오류", f"시리얼 포트를 열 수 없습니다: {e}")
        else:
            messagebox.showwarning("시리얼 포트 선택", "시리얼 포트를 선택하세요.")

    def close_serial(self):
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
            messagebox.showinfo("시리얼 연결", "시리얼 포트가 닫혔습니다")
        else:
            messagebox.showwarning("시리얼 연결", "시리얼 포트가 열려 있지 않습니다")

    def setup_robot_arm(self):
        arm_control = Frame(self.master, width=300)  # 설정한 폭
        arm_control.pack()

        arm_label = Label(arm_control, text="Robot Arm Components", bg="yellow", padx=100)
        arm_label.pack()

        self.frame_scale = self.setup_joint_control(arm_control, "Base")
        self.base_scale = self.setup_joint_control(arm_control, "Shoulder")
        self.shoulder_scale = self.setup_joint_control(arm_control, "upper arm")
        self.elbow_scale = self.setup_joint_control(arm_control, "fore arm")

        arm_go_button = Button(arm_control, text="Move Robot Arm", height=2, width=15, bg="black", fg="white", command=self.move_robot_arm, padx=100)
        arm_go_button.pack()

        arm_rem_button = Button(arm_control, text="show remeber action", height=2, width=15, bg="black", fg="white", command=self.action_rem_arm, padx=100)
        arm_rem_button.pack()
        
    def setup_joint_control(self, parent, joint_name):
        joint_label = Label(parent, text=joint_name, width=15, bg="green", fg="yellow", padx=100)
        joint_label.pack()

        joint_scale = Scale(parent, from_=1, to=175, length=300, orient='horizontal')
        joint_scale.pack()

        return joint_scale
    
    def action_rem_arm(self):  #큐 사용해서 기억한 동작 수행
        def check_queue():
            while not self.action_queue.empty():
                action = self.action_queue.get()
                action_bytes = action.encode('utf-8')
                self.ser.write(action_bytes)
                print("Action : ", action)
                print("size : " , self.action_queue.qsize())
            self.master.after(7000, self.check_next_action)
        check_queue()
        
    def check_next_action(self):
        if not self.action_queue.empty():
            self.action_rem_arm

    def move_robot_arm(self):   #로봇 이동 함수, 큐에 데이터 저장
        command = f"{self.frame_scale.get()},{self.base_scale.get()},{self.shoulder_scale.get()},{self.elbow_scale.get()}d"
        command_bytes = command.encode('utf-8')
        self.ser.write(command_bytes)
        self.action_queue.put(command)
        print(command)
        print("size : " , self.action_queue.qsize())

    def setup_spacer_frame(self, **kwargs):
        spacer_frame = Frame(self.master, width=300)  # 설정한 폭
        spacer_frame.pack(**kwargs)

if __name__ == "__main__":
    root = Tk()
    app = RobotArmControl(root)
    root.mainloop()

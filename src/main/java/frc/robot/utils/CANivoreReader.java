package frc.robot.utils;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.CANBus.CANBusStatus;
public class CANivoreReader {
    private CANBusStatus currentStatus = new CANBusStatus();

    private final CANBus canBus;

    public CANivoreReader(CANBus canBus) {
        this.canBus = canBus;

        Thread readThread = new Thread(this::updateStatus);
        readThread.start();
    }

    private void updateStatus() {
        while (true) {
            var statusCapture = canBus.getStatus();

            synchronized (this) {
                currentStatus = statusCapture;//Sync on write
            }

            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    public synchronized CANBusStatus getStatus() {
        return currentStatus;//Sync on read
    }
}

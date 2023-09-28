
class teleopfoda {
    DcMotor hexmotor = null;
    
    @Override
    public void init() {
      hexmotor = hardwareMap.get(DcMotor.class, "my_motor");
    }

    @Override
    public void loop() {
        hexmotor.setPower(gamepad1.right_bumper-gamepad1.left_bumper);
    }
}

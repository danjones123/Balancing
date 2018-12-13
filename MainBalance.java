package Balancing;

public class MainBalance {

	public static void main(String[] args) {
		TheBalancingAct balanceRobot = new TheBalancingAct();
		balanceRobot.start();
		balanceRobot.limitSpeed(2);
		balanceRobot.fallen();
		
	}

}

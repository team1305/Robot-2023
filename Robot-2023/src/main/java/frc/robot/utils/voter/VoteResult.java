package frc.robot.utils.voter;

public class VoteResult {
    private final int[] m_winners;
    private final int[] m_outliers;

    public VoteResult(int[] winners, int[] outliers){
        m_winners = winners;
        m_outliers = outliers;
    }

    public int[] getWinners(){
        return m_winners;
    }

    public int[] getOutliers(){
        return m_outliers;
    }
}
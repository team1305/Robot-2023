package frc.robot.utils.voter;

public class Ballot {
    private final double m_value;
    private final int m_voteCount;

    public Ballot(double value, int voteCount){
        m_value = value;
        m_voteCount = voteCount;
    }

    public double getValue(){
        return m_value;
    }

    public int getVoteCount(){
        return m_voteCount;
    }
}

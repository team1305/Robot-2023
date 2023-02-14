package frc.robot.utils;

public class Ballot {
    private final double m_value;
    private final int m_votes;

    public Ballot(double value, int votes){
        m_value = value;
        m_votes = votes;
    }

    public double getValue(){
        return m_value;
    }

    public int getVotes(){
        return m_votes;
    }
}

package frc.robot.utils.voter;

public class VoteTally {
    private final int m_winningVoteCount;
    private final int m_frequency;

    public VoteTally(int winningVoteCount, int frequency){
        m_winningVoteCount = winningVoteCount;
        m_frequency = frequency;
    }
    
    public int getWinningVoteCount(){
        return m_winningVoteCount;
    }

    public int getFrequency(){
        return m_frequency;
    }
}
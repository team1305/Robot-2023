package frc.robot.utils.voter;

public class TMRDoubleVoter {

    private final double m_threshold;
    private final double[] m_values;

    private int[] m_outliers;

    public TMRDoubleVoter(
        double threshold,
        double value1,
        double value2,
        double... valueN
    ){
        m_threshold = threshold;
        m_values = new double[valueN.length + 2];
        m_values[0] = value1;
        m_values[1] = value2;
        System.arraycopy(valueN, 0, m_values, 2, valueN.length);
    }

    /// Performs a voting operation on the values inputed.
    /// This will vote on values that are within a threshold.
    /// Winners of the vote are averaged out and the average value returned.
    /// Outliers are stored in the outliers array.
    public double vote(){
        Ballot[] ballots = new Ballot[m_values.length];

        signBallots(ballots);

        VoteTally tally = getTally(ballots);

        VoteResult result = getResult(tally, ballots);

        m_outliers = result.getOutliers();

        return averageWinners(result.getWinners());
    }

    /// Returns the outliers of the voting operation.
    public int[] getOutliers(){
        return m_outliers;
    }

    /// A helper method that creates a Ballot for each value.
    /// The ballot contains the value and the number of votes that it gets.
    private void signBallots(Ballot[] ballots){
        for(int i = 0; i < m_values.length; ++i){
            int votes = 0;

            for(int j = 0; j < m_values.length; ++j){
                if(Math.abs(m_values[i] - m_values[j]) < m_threshold){
                    ++votes;
                }
            }

            ballots[i] = new Ballot(m_values[i], votes);
        }
    }

    /// A helper method that tallies the results of the ballots.
    /// The tally will contain the how many votes the winning ballot(s) received
    /// as well as the frequency in which that count occured (number of first place ties)
    private VoteTally getTally(Ballot[] ballots){
        int winningVoteCount = 0;
        int frequency = 1;

        for(int i = 0; i < ballots.length; ++i){
            if(ballots[i].getVoteCount() > winningVoteCount){
                winningVoteCount = ballots[i].getVoteCount();
                frequency = 1;
            }
            else if(ballots[i].getVoteCount() == winningVoteCount){
                ++frequency;
            }
        }
        
        return new VoteTally(winningVoteCount, frequency);
    }

    /// A helper method that gets the result of the voting.
    /// It will return a VoteResult which includes the indexes of the winners and the indexes of the outliers.
    private VoteResult getResult(VoteTally result, Ballot[] ballots){

        int numWinners = result.getFrequency();
        int numOutliers = m_values.length - numWinners;

        int[] winners = new int[numWinners];
        int[] outliers = new int[numOutliers];
        int j = 0;  // An iterator for the winners list
        int k = 0;  // an iterator for the outliers list

        for (int i = 0; i < ballots.length; ++i){
            if(ballots[i].getVoteCount() == result.getWinningVoteCount()){
                winners[j] = i;
                ++j;
            }
            else{
                outliers[k] = i;
                ++k;
            }
        }

        return new VoteResult(winners, outliers);
    }

    private double averageWinners(int[] winners){
        double sum = 0;

        for(int i = 0; i < winners.length; ++i){
            sum = sum + m_values[winners[i]];
        }

        return sum/winners.length;
    }
}
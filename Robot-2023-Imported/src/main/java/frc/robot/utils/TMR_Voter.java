package frc.robot.utils;

public class TMR_Voter {

    private final double m_threshold;
    private final double[] m_values;

    public TMR_Voter(
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

        public double vote(){
            Ballot[] ballots = new Ballot[m_values.length];
            
            for(int i = 0; i < m_values.length; ++i){
                int votes = 0;

                for(int j = 0; j < m_values.length; ++j){
                    if(Math.abs(m_values[i] - m_values[j]) < m_threshold){
                        ++votes;
                    }
                }

                ballots[i] = new Ballot(m_values[i], votes);
            }
            
            int mostVotes = 0;
            int frequency = 1;

            for(int i = 0; i < ballots.length; ++i){
                if(ballots[i].getVotes() > mostVotes){
                    mostVotes = ballots[i].getVotes();
                    frequency = 1;
                }
                else if(ballots[i].getVotes() == mostVotes){
                    ++frequency;
                }
            }

            double[] winners = new double[frequency];
            int j = 0;

            for (int i = 0; i < ballots.length; ++i){
                if(ballots[i].getVotes() == mostVotes){
                    winners[j] = ballots[i].getValue();
                    ++j;
                }
            }

            double sum = 0;

            for(int i = 0; i < winners.length; ++i){
                sum = sum + winners[i];
            }

            return sum/winners.length;
        }
}

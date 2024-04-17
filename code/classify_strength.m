% Classify interaction strengths and calculate classification accuracy

%% Transformation of the SAB matrix into an interaction strength classification result matrix.
% 94.5 percentage points corresponds to an interaction probability of 0.7
% 61 percentage points corresponds to an interaction probability of 0.3.
p945_SAB = prctile(SAB(:),94.5);
p61_SAB = prctile(SAB(:),61);

% Calculate the classification result matrix of the testset.Strong level is 3,weak level is 2.
Strength_SAB = testSAB;
Strength_SAB(testSAB >= p945_SAB) = 3;
Strength_SAB(testSAB <= p61_SAB) = 1;
Strength_SAB(testSAB > p61_SAB & testSAB < p945_SAB) = 2;

% This code snippet is only used for the TTCP because the TTCP is negatively correlated with interaction probability.
% p55_SAB = prctile(SAB(:),5.5);
% p39_SAB = prctile(SAB(:),39);
% Strength_SAB = testSAB;
% Strength_SAB(testSAB <= p55_SAB) = 3;
% Strength_SAB(testSAB >= p39_SAB) = 1;
% Strength_SAB(testSAB < p39_SAB & testSAB > p55_SAB) = 2;

%% Transformation of human judgment data matrix into a truth matrix for classification results.
Strength_prob = testprob;
Strength_prob(testprob >= 0.7) = 3;
Strength_prob(testprob <= 0.3) = 1;
Strength_prob(testprob > 0.3 & testprob < 0.7) = 2;

%% Calculate classification accuracy
Strength_prob = reshape(Strength_prob,[],1);
Strength_SAB = reshape(Strength_SAB,[],1);
accuracy = sum(Strength_prob == Strength_SAB) ./ numel(Strength_SAB);

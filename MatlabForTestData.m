%%
clc
clear all
close all

%%

clc
clear all
close all

DataTestSize = 6;
RecordedTestFiles = 4;

CorrectMessages = 1;
IncorrectMessages = 2;
IncorrectFormat = 3;
TotalMessages = 4;
CorrectFormat_Result_Index = 5;
CorrectMessage_Result_Index = 6;
% File Handling
formatSpec = '%f';

DFT_CalcTime_File = fopen('DFT_buffer_CalculationSpeed.txt','r');
FFT_CalcTime_File = fopen('FFT_buffer_CalculationSpeed.txt','r');
Goertzel_CalcTime_File = fopen('Goertzel_buffer_CalculationSpeed.txt','r');

DFT_CorrectAndFail_File = fopen('DFT_correctAndFailFile.txt','r');
FFT_CorrectAndFail_File = fopen('FFT_correctAndFailFile.txt','r');
Goertzel_CorrectAndFail_File = fopen('Goertzel_correctAndFailFile.txt','r');

DFT_TimeTestData = fscanf(DFT_CalcTime_File,formatSpec);
FFT_TimeTestData = fscanf(FFT_CalcTime_File,formatSpec);
Goertzel_TimeTestData = fscanf(Goertzel_CalcTime_File,formatSpec);

DFT_CorrectAndFailData = fscanf(DFT_CorrectAndFail_File,formatSpec);
FFT_CorrectAndFailData = fscanf(FFT_CorrectAndFail_File,formatSpec);
Goertzel_CorrectAndFailData = fscanf(Goertzel_CorrectAndFail_File,formatSpec);

% Time
DFT_Calc_Speed = sum(DFT_TimeTestData(1:RecordedTestFiles))/RecordedTestFiles;
FFT_Calc_Speed = sum(FFT_TimeTestData(1:RecordedTestFiles))/RecordedTestFiles;
Goertzel_Calc_Speed = sum(Goertzel_TimeTestData(1:RecordedTestFiles))/RecordedTestFiles;

SpeedResults_DFT_VS_FFT = [DFT_Calc_Speed,FFT_Calc_Speed];
SpeedResults_FFT_VS_Goertzel = [FFT_Calc_Speed,Goertzel_Calc_Speed]*1000;

Categories_DFT_FFT = ["DFT" "FFT"];
Categories_FFT_Goertzel = ["FFT" "Goertzel"];

figure('Name','DFT speed comparison')
subplot(1,2,1)
piechart(SpeedResults_DFT_VS_FFT,Categories_DFT_FFT,LabelStyle="namedata");
title('DFT compared with FFT (ms)')
subplot(1,2,2)
piechart(SpeedResults_FFT_VS_Goertzel,Categories_FFT_Goertzel,LabelStyle="namedata");
title('FFT compared with Goertzel (us)')

% Presicion DFT 
DFT_CorrectAndFail_ShortEcho = DFT_CorrectAndFailData(CorrectMessages:IncorrectFormat);
DFT_CorrectAndFail_ShortNoEcho = DFT_CorrectAndFailData(CorrectMessages+DataTestSize:IncorrectFormat+DataTestSize);
DFT_CorrectAndFail_LongEcho = DFT_CorrectAndFailData(CorrectMessages+DataTestSize*2:IncorrectFormat+DataTestSize*2);
DFT_CorrectAndFail_LongNoEcho = DFT_CorrectAndFailData(CorrectMessages+DataTestSize*3:IncorrectFormat+DataTestSize*3);

Categories_CorrectIncorrect = [ "Correct" "Incorrect message" "Incorrect format"];


figure('Name','DFT Precision')
subplot(2,2,1)
piechart(DFT_CorrectAndFail_ShortEcho,Categories_CorrectIncorrect,LabelStyle="namedata")
title('Short distance with Echo')
subplot(2,2,2)
piechart(DFT_CorrectAndFail_ShortNoEcho,Categories_CorrectIncorrect,LabelStyle="namedata")
title('Short distance without Echo')
subplot(2,2,3)
piechart(DFT_CorrectAndFail_LongEcho,Categories_CorrectIncorrect,LabelStyle="namedata")
title('Long distance with Echo')
subplot(2,2,4)
piechart(DFT_CorrectAndFail_LongNoEcho,Categories_CorrectIncorrect,LabelStyle="namedata")
title('Long distance without Echo')



% Presicion FFT
FFT_CorrectAndFail_ShortEcho = FFT_CorrectAndFailData(CorrectMessages:IncorrectFormat);
FFT_CorrectAndFail_ShortNoEcho = FFT_CorrectAndFailData(CorrectMessages+DataTestSize:IncorrectFormat+DataTestSize);
FFT_CorrectAndFail_LongEcho = FFT_CorrectAndFailData(CorrectMessages+DataTestSize*2:IncorrectFormat+DataTestSize*2);
FFT_CorrectAndFail_LongNoEcho = FFT_CorrectAndFailData(CorrectMessages+DataTestSize*3:IncorrectFormat+DataTestSize*3);

Categories_CorrectIncorrect = ["Correct" "Incorrect message" "Incorrect format"];


figure('Name','FFT Precision')
subplot(2,2,1)
piechart(FFT_CorrectAndFail_ShortEcho,Categories_CorrectIncorrect,LabelStyle="namedata")
title('Short distance with Echo')
subplot(2,2,2)
piechart(FFT_CorrectAndFail_ShortNoEcho,Categories_CorrectIncorrect,LabelStyle="namedata")
title('Short distance without Echo')
subplot(2,2,3)
piechart(FFT_CorrectAndFail_LongEcho,Categories_CorrectIncorrect,LabelStyle="namedata")
title('Long distance with Echo')
subplot(2,2,4)
piechart(FFT_CorrectAndFail_LongNoEcho,Categories_CorrectIncorrect,LabelStyle="namedata")
title('Long distance without Echo')

% Presicion Goertzel
Goertzel_CorrectAndFail_ShortEcho = Goertzel_CorrectAndFailData(CorrectMessages:IncorrectFormat);
Goertzel_CorrectAndFail_ShortNoEcho = Goertzel_CorrectAndFailData(CorrectMessages+DataTestSize:IncorrectFormat+DataTestSize);
Goertzel_CorrectAndFail_LongEcho = Goertzel_CorrectAndFailData(CorrectMessages+DataTestSize*2:IncorrectFormat+DataTestSize*2);
Goertzel_CorrectAndFail_LongNoEcho = Goertzel_CorrectAndFailData(CorrectMessages+DataTestSize*3:IncorrectFormat+DataTestSize*3);

Categories_CorrectIncorrect = ["Correct" "Incorrect message" "Incorrect format"];


figure('Name','Goertzel Precision')
subplot(2,2,1)
piechart(Goertzel_CorrectAndFail_ShortEcho,Categories_CorrectIncorrect,LabelStyle="namedata")
title('Short distance with Echo')
subplot(2,2,2)
piechart(Goertzel_CorrectAndFail_ShortNoEcho,Categories_CorrectIncorrect,LabelStyle="namedata")
title('Short distance without Echo')
subplot(2,2,3)
piechart(Goertzel_CorrectAndFail_LongEcho,Categories_CorrectIncorrect,LabelStyle="namedata")
title('Long distance with Echo')
subplot(2,2,4)
piechart(Goertzel_CorrectAndFail_LongNoEcho,Categories_CorrectIncorrect,LabelStyle="namedata")
title('Long distance without Echo')


% Correct messages

DFT_CorrectMessages_ShortEcho = DFT_CorrectAndFailData(CorrectMessage_Result_Index);
DFT_CorrectMessages_ShortNoEcho = DFT_CorrectAndFailData(CorrectMessage_Result_Index+DataTestSize);
DFT_CorrectMessages_LongEcho = DFT_CorrectAndFailData(CorrectMessage_Result_Index+DataTestSize*2);
DFT_CorrectMessages_LongNoEcho = DFT_CorrectAndFailData(CorrectMessage_Result_Index+DataTestSize*3);

FFT_CorrectMessages_ShortEcho = FFT_CorrectAndFailData(CorrectMessage_Result_Index);
FFT_CorrectMessages_ShortNoEcho = FFT_CorrectAndFailData(CorrectMessage_Result_Index+DataTestSize*1);
FFT_CorrectMessages_LongEcho = FFT_CorrectAndFailData(CorrectMessage_Result_Index+DataTestSize*2);
FFT_CorrectMessages_LongNoEcho = FFT_CorrectAndFailData(CorrectMessage_Result_Index+DataTestSize*3);

Goertzel_CorrectMessages_ShortEcho = Goertzel_CorrectAndFailData(CorrectMessage_Result_Index);
Goertzel_CorrectMessages_ShortNoEcho = Goertzel_CorrectAndFailData(CorrectMessage_Result_Index+DataTestSize);
Goertzel_CorrectMessages_LongEcho = Goertzel_CorrectAndFailData(CorrectMessage_Result_Index+DataTestSize*2);
Goertzel_CorrectMessages_LongNoEcho = Goertzel_CorrectAndFailData(CorrectMessage_Result_Index+DataTestSize*3);

Categories_CorrectMessages = ["DFT" "FFT" "Goertzel"];

CorrectMessage_Short_Echo = [DFT_CorrectMessages_ShortEcho FFT_CorrectMessages_ShortEcho Goertzel_CorrectMessages_ShortEcho];
CorrectMessage_Short_NoEcho = [DFT_CorrectMessages_ShortNoEcho FFT_CorrectMessages_ShortNoEcho Goertzel_CorrectMessages_ShortNoEcho];
CorrectMessage_Long_Echo = [DFT_CorrectMessages_LongEcho FFT_CorrectMessages_LongEcho Goertzel_CorrectMessages_LongEcho];
CorrectMessage_Long_NoEcho = [DFT_CorrectMessages_LongNoEcho FFT_CorrectMessages_LongNoEcho Goertzel_CorrectMessages_LongNoEcho];

figure('Name','Percentage of correct messages')
subplot(2,2,1)
piechart(CorrectMessage_Short_Echo,Categories_CorrectMessages,LabelStyle="namedata")
title('Short distance with Echo')
subplot(2,2,2)
piechart(CorrectMessage_Short_NoEcho,Categories_CorrectMessages,LabelStyle="namedata")
title('Short distance without Echo')
subplot(2,2,3)
piechart(CorrectMessage_Long_Echo,Categories_CorrectMessages,LabelStyle="namedata")
title('Long distance with Echo')
subplot(2,2,4)
piechart(CorrectMessage_Long_NoEcho,Categories_CorrectMessages,LabelStyle="namedata")
title('Long distance without Echo')


% Correct format

DFT_CorrectFormat_ShortEcho = DFT_CorrectAndFailData(CorrectFormat_Result_Index);
DFT_CorrectFormat_ShortNoEcho = DFT_CorrectAndFailData(CorrectFormat_Result_Index+DataTestSize);
DFT_CorrectFormat_LongEcho = DFT_CorrectAndFailData(CorrectFormat_Result_Index+DataTestSize*2);
DFT_CorrectFormat_LongNoEcho = DFT_CorrectAndFailData(CorrectFormat_Result_Index+DataTestSize*3);

FFT_CorrectFormat_ShortEcho = FFT_CorrectAndFailData(CorrectFormat_Result_Index);
FFT_CorrectFormat_ShortNoEcho = FFT_CorrectAndFailData(CorrectFormat_Result_Index+DataTestSize);
FFT_CorrectFormat_LongEcho = FFT_CorrectAndFailData(CorrectFormat_Result_Index+DataTestSize*2);
FFT_CorrectFormat_LongNoEcho = FFT_CorrectAndFailData(CorrectFormat_Result_Index+DataTestSize*3);

Goertzel_CorrectFormat_ShortEcho = Goertzel_CorrectAndFailData(CorrectFormat_Result_Index);
Goertzel_CorrectFormat_ShortNoEcho = Goertzel_CorrectAndFailData(CorrectFormat_Result_Index+DataTestSize);
Goertzel_CorrectFormat_LongEcho = Goertzel_CorrectAndFailData(CorrectFormat_Result_Index+DataTestSize*2);
Goertzel_CorrectFormat_LongNoEcho = Goertzel_CorrectAndFailData(CorrectFormat_Result_Index+DataTestSize*3);


Categories_CorrectFormat = ["DFT" "FFT" "Goertzel"];

CorrectFormat_Short_Echo = [DFT_CorrectFormat_ShortEcho FFT_CorrectFormat_ShortEcho Goertzel_CorrectFormat_ShortEcho];
CorrectFormat_Short_NoEcho = [DFT_CorrectFormat_ShortNoEcho FFT_CorrectFormat_ShortNoEcho Goertzel_CorrectFormat_ShortNoEcho];
CorrectFormat_Long_Echo = [DFT_CorrectFormat_LongEcho FFT_CorrectFormat_LongEcho Goertzel_CorrectFormat_LongEcho];
CorrectFormat_Long_NoEcho = [DFT_CorrectFormat_LongNoEcho FFT_CorrectFormat_LongNoEcho Goertzel_CorrectFormat_LongNoEcho];

figure('Name','Percentage of messages with correct format')
subplot(2,2,1)
piechart(CorrectFormat_Short_Echo,Categories_CorrectFormat,LabelStyle="namedata")
title('Short distance with Echo')
subplot(2,2,2)
piechart(CorrectFormat_Short_NoEcho,Categories_CorrectFormat,LabelStyle="namedata")
title('Short distance without Echo')
subplot(2,2,3)
piechart(CorrectFormat_Long_Echo,Categories_CorrectFormat,LabelStyle="namedata")
title('Long distance with Echo')
subplot(2,2,4)
piechart(CorrectFormat_Long_NoEcho,Categories_CorrectFormat,LabelStyle="namedata")
title('Long distance without Echo')


% 1 Channel vs 2 Channel Goertzel

Goertzel_CorrectFormat_Short_1Channel = Goertzel_CorrectAndFailData(CorrectMessages+DataTestSize*4:IncorrectFormat+DataTestSize*4);
Goertzel_CorrectFormat_Short_2Channel = Goertzel_CorrectAndFailData(CorrectMessages+DataTestSize*5:IncorrectFormat+DataTestSize*5);

figure('Name','Goertzel channel Precision')
subplot(1,2,1)
piechart(Goertzel_CorrectFormat_Short_1Channel,Categories_CorrectIncorrect,LabelStyle="namedata")
title('1 Channel')
subplot(1,2,2)
piechart(Goertzel_CorrectFormat_Short_2Channel,Categories_CorrectIncorrect,LabelStyle="namedata")
title('2 channel')
subplot(2,2,3)



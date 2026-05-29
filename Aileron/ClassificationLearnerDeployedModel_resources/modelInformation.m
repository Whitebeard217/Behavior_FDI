function modelInfoJSON = modelInformation()
modelInfoStruct = struct();
modelInfoStruct.About = 'Predict on new data using a trained model exported from Classification Learner R2024a.';
modelInfoStruct.Version = '1.0';
modelInfoStruct.InputFormat = 'table';
modelInfoStruct.InputColumnOrder = {'R', 'dR', 'ddR', 'x0_5F', 'x0_8F', 'fault'};
modelInfoStruct.InputColumnFormat = {'double', 'double', 'double', 'double', 'double', 'double'};
modelInfoStruct.OutputFormat = 'double';
modelInfoJSON = jsonencode(modelInfoStruct);
end